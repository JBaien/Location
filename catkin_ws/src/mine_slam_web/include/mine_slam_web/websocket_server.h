#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace mine_slam_web {

class WebSocketSession;

class WebSocketServer {
 public:
  WebSocketServer(std::uint16_t port, bool binary_messages);
  ~WebSocketServer();

  WebSocketServer(const WebSocketServer&) = delete;
  WebSocketServer& operator=(const WebSocketServer&) = delete;

  void start();
  void stop();

  std::size_t clientCount() const;
  void setClientConnectedCallback(std::function<void()> callback);
  void broadcast(std::shared_ptr<const std::vector<std::uint8_t>> payload);
  void broadcastText(const std::string& payload);

 private:
  void doAccept();
  void addSession(const std::shared_ptr<WebSocketSession>& session);
  void removeClosedSessions();

  std::uint16_t port_;
  bool binary_messages_;
  mutable std::mutex sessions_mutex_;
  std::vector<std::weak_ptr<WebSocketSession>> sessions_;
  std::mutex callback_mutex_;
  std::function<void()> client_connected_callback_;
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace mine_slam_web

