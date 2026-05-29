#include "mine_slam_web/websocket_server.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <iostream>
#include <sstream>
#include <utility>

#include <openssl/sha.h>

namespace mine_slam_web {
using tcp = boost::asio::ip::tcp;

namespace {

std::string trim(std::string value) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string base64Encode(const std::array<unsigned char, 20>& bytes) {
  static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(28);
  std::size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const unsigned int value = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    out.push_back(kTable[(value >> 18) & 0x3F]);
    out.push_back(kTable[(value >> 12) & 0x3F]);
    out.push_back(kTable[(value >> 6) & 0x3F]);
    out.push_back(kTable[value & 0x3F]);
  }
  if (i < bytes.size()) {
    unsigned int value = bytes[i] << 16;
    out.push_back(kTable[(value >> 18) & 0x3F]);
    if (i + 1 < bytes.size()) {
      value |= bytes[i + 1] << 8;
      out.push_back(kTable[(value >> 12) & 0x3F]);
      out.push_back(kTable[(value >> 6) & 0x3F]);
      out.push_back('=');
    } else {
      out.push_back(kTable[(value >> 12) & 0x3F]);
      out.push_back('=');
      out.push_back('=');
    }
  }
  return out;
}

std::string websocketAcceptKey(const std::string& client_key) {
  static constexpr char kGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  const std::string input = client_key + kGuid;
  std::array<unsigned char, 20> bytes{};
  SHA1(reinterpret_cast<const unsigned char*>(input.data()), input.size(), bytes.data());
  return base64Encode(bytes);
}

std::string extractWebSocketKey(const std::string& request) {
  std::istringstream stream(request);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string name = toLower(trim(line.substr(0, colon)));
    if (name == "sec-websocket-key") {
      return trim(line.substr(colon + 1));
    }
  }
  return {};
}

std::shared_ptr<const std::vector<std::uint8_t>> makeFrame(const std::vector<std::uint8_t>& payload, bool binary) {
  auto frame = std::make_shared<std::vector<std::uint8_t>>();
  frame->reserve(payload.size() + 14);
  frame->push_back(binary ? 0x82 : 0x81);

  const auto size = payload.size();
  if (size <= 125) {
    frame->push_back(static_cast<std::uint8_t>(size));
  } else if (size <= 0xFFFF) {
    frame->push_back(126);
    frame->push_back(static_cast<std::uint8_t>((size >> 8) & 0xFF));
    frame->push_back(static_cast<std::uint8_t>(size & 0xFF));
  } else {
    frame->push_back(127);
    for (int shift = 56; shift >= 0; shift -= 8) {
      frame->push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(size) >> shift) & 0xFF));
    }
  }
  frame->insert(frame->end(), payload.begin(), payload.end());
  return frame;
}

}  // namespace

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
 public:
  WebSocketSession(tcp::socket socket, boost::asio::io_service& io_service, bool binary_messages,
                   std::function<void()> accepted_callback)
      : socket_(std::move(socket)),
        strand_(io_service),
        binary_messages_(binary_messages),
        accepted_callback_(std::move(accepted_callback)) {}

  void start() {
    boost::asio::async_read_until(socket_, read_buffer_, "\r\n\r\n",
                                  strand_.wrap([self = shared_from_this()](const boost::system::error_code& ec,
                                                                           std::size_t bytes_transferred) {
                                    self->handleHandshake(ec, bytes_transferred);
                                  }));
  }

  bool closed() const {
    return closed_.load();
  }

  void send(std::shared_ptr<const std::vector<std::uint8_t>> payload) {
    const auto frame = makeFrame(*payload, binary_messages_);
    strand_.post([self = shared_from_this(), frame]() {
      const bool writing = !self->write_queue_.empty();
      self->write_queue_.push_back(std::move(frame));
      if (!writing) {
        self->doWrite();
      }
    });
  }

 private:
  void handleHandshake(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (ec) {
      closed_.store(true);
      return;
    }

    std::string request(bytes_transferred, '\0');
    read_buffer_.sgetn(&request[0], static_cast<std::streamsize>(bytes_transferred));
    const std::string key = extractWebSocketKey(request);
    if (key.empty()) {
      closed_.store(true);
      return;
    }

    const std::string response =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        websocketAcceptKey(key) +
        "\r\n"
        "Server: mine_slam_web\r\n"
        "\r\n";

    auto response_ptr = std::make_shared<std::string>(response);
    boost::asio::async_write(socket_, boost::asio::buffer(*response_ptr),
                             strand_.wrap([self = shared_from_this(), response_ptr](
                                              const boost::system::error_code& write_ec, std::size_t) {
                               if (write_ec) {
                                 self->closed_.store(true);
                                 return;
                               }
                               self->doRead();
                               if (self->accepted_callback_) {
                                 self->accepted_callback_();
                               }
                             }));
  }

  void doRead() {
    socket_.async_read_some(boost::asio::buffer(read_scratch_),
                            strand_.wrap([self = shared_from_this()](const boost::system::error_code& ec,
                                                                     std::size_t) {
                              if (ec) {
                                self->closed_.store(true);
                                return;
                              }
                              self->doRead();
                            }));
  }

  void doWrite() {
    if (write_queue_.empty()) {
      return;
    }
    boost::asio::async_write(socket_, boost::asio::buffer(*write_queue_.front()),
                             strand_.wrap([self = shared_from_this()](const boost::system::error_code& ec,
                                                                      std::size_t) {
                               if (ec) {
                                 self->closed_.store(true);
                                 self->write_queue_.clear();
                                 return;
                               }
                               self->write_queue_.erase(self->write_queue_.begin());
                               self->doWrite();
                             }));
  }

  tcp::socket socket_;
  boost::asio::io_service::strand strand_;
  bool binary_messages_;
  boost::asio::streambuf read_buffer_;
  std::array<std::uint8_t, 1024> read_scratch_{};
  std::vector<std::shared_ptr<const std::vector<std::uint8_t>>> write_queue_;
  std::atomic<bool> closed_{false};
  std::function<void()> accepted_callback_;
};

WebSocketServer::WebSocketServer(std::uint16_t port, bool binary_messages)
    : port_(port), binary_messages_(binary_messages) {}

WebSocketServer::~WebSocketServer() {
  stop();
}

void WebSocketServer::start() {
  if (running_.exchange(true)) {
    return;
  }
  tcp::endpoint endpoint(tcp::v4(), port_);
  acceptor_ = std::make_unique<tcp::acceptor>(io_service_);
  boost::system::error_code ec;
  acceptor_->open(endpoint.protocol(), ec);
  if (ec) {
    throw std::runtime_error("open websocket acceptor failed: " + ec.message());
  }
  acceptor_->set_option(boost::asio::socket_base::reuse_address(true), ec);
  if (ec) {
    throw std::runtime_error("set websocket reuse_address failed: " + ec.message());
  }
  acceptor_->bind(endpoint, ec);
  if (ec) {
    throw std::runtime_error("bind websocket port " + std::to_string(port_) + " failed: " + ec.message());
  }
  acceptor_->listen(boost::asio::socket_base::max_connections, ec);
  if (ec) {
    throw std::runtime_error("listen websocket port " + std::to_string(port_) + " failed: " + ec.message());
  }

  doAccept();
  io_thread_ = std::thread([this]() { io_service_.run(); });
}

void WebSocketServer::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  io_service_.post([this]() {
    if (acceptor_) {
      boost::system::error_code ec;
      acceptor_->close(ec);
    }
  });
  io_service_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

std::size_t WebSocketServer::clientCount() const {
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  std::size_t count = 0;
  for (const auto& weak_session : sessions_) {
    const auto session = weak_session.lock();
    if (session && !session->closed()) {
      ++count;
    }
  }
  return count;
}

void WebSocketServer::setClientConnectedCallback(std::function<void()> callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  client_connected_callback_ = std::move(callback);
}

void WebSocketServer::broadcast(std::shared_ptr<const std::vector<std::uint8_t>> payload) {
  removeClosedSessions();
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  for (const auto& weak_session : sessions_) {
    const auto session = weak_session.lock();
    if (session && !session->closed()) {
      session->send(payload);
    }
  }
}

void WebSocketServer::broadcastText(const std::string& payload) {
  auto bytes = std::make_shared<std::vector<std::uint8_t>>(payload.begin(), payload.end());
  broadcast(bytes);
}

void WebSocketServer::doAccept() {
  auto socket = std::make_shared<tcp::socket>(io_service_);
  acceptor_->async_accept(*socket, [this, socket](const boost::system::error_code& ec) {
    if (!ec) {
      std::function<void()> callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = client_connected_callback_;
      }
      const auto session =
          std::make_shared<WebSocketSession>(std::move(*socket), io_service_, binary_messages_, std::move(callback));
      addSession(session);
      session->start();
    }
    if (running_.load()) {
      doAccept();
    }
  });
}

void WebSocketServer::addSession(const std::shared_ptr<WebSocketSession>& session) {
  removeClosedSessions();
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  sessions_.push_back(session);
}

void WebSocketServer::removeClosedSessions() {
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  sessions_.erase(std::remove_if(sessions_.begin(), sessions_.end(),
                                 [](const std::weak_ptr<WebSocketSession>& weak_session) {
                                   const auto session = weak_session.lock();
                                   return !session || session->closed();
                                 }),
                  sessions_.end());
}

}  // namespace mine_slam_web

