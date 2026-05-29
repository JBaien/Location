#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <target_localizer/EquipmentState.h>

namespace target_localizer {
namespace {

using boost::asio::ip::tcp;

struct ModbusDeviceConfig {
    std::string host = "127.0.0.1";
    int port = 502;
    int unit_id = 1;
    int timeout_ms = 500;
};

struct RegisterConfig {
    int address = 0;
    double scale = 1.0;
};

std::int16_t signedRegister(std::uint16_t value) {
    return static_cast<std::int16_t>(value);
}

class ModbusClient {
public:
    explicit ModbusClient(boost::asio::io_context& io) : io_(io), socket_(io) {}

    bool readHoldingRegisters(const ModbusDeviceConfig& config,
                              int start_address,
                              int count,
                              std::vector<std::uint16_t>& out) {
        out.clear();
        boost::system::error_code ec;
        if (!socket_.is_open()) {
            tcp::resolver resolver(io_);
            const auto endpoints = resolver.resolve(config.host,
                                                    std::to_string(config.port),
                                                    ec);
            if (ec) {
                return false;
            }
            boost::asio::connect(socket_, endpoints, ec);
            if (ec) {
                socket_.close();
                return false;
            }
        }

        const std::uint16_t transaction = ++transaction_id_;
        std::array<std::uint8_t, 12> request{};
        request[0] = static_cast<std::uint8_t>(transaction >> 8);
        request[1] = static_cast<std::uint8_t>(transaction & 0xff);
        request[4] = 0;
        request[5] = 6;
        request[6] = static_cast<std::uint8_t>(config.unit_id);
        request[7] = 3;
        request[8] = static_cast<std::uint8_t>(start_address >> 8);
        request[9] = static_cast<std::uint8_t>(start_address & 0xff);
        request[10] = static_cast<std::uint8_t>(count >> 8);
        request[11] = static_cast<std::uint8_t>(count & 0xff);
        boost::asio::write(socket_, boost::asio::buffer(request), ec);
        if (ec) {
            socket_.close();
            return false;
        }

        std::array<std::uint8_t, 260> response{};
        const std::size_t n = socket_.read_some(boost::asio::buffer(response), ec);
        if (ec || n < 9 || response[7] != 3) {
            socket_.close();
            return false;
        }
        const int byte_count = response[8];
        if (n < static_cast<std::size_t>(9 + byte_count) ||
            byte_count < count * 2) {
            socket_.close();
            return false;
        }
        out.reserve(count);
        for (int i = 0; i < count; ++i) {
            const int offset = 9 + i * 2;
            out.push_back(static_cast<std::uint16_t>(
                (static_cast<std::uint16_t>(response[offset]) << 8) |
                static_cast<std::uint16_t>(response[offset + 1])));
        }
        return true;
    }

    void close() {
        boost::system::error_code ignored;
        socket_.close(ignored);
    }

private:
    boost::asio::io_context& io_;
    tcp::socket socket_;
    std::uint16_t transaction_id_ = 0;
};

class ModbusSensorReferenceNode {
public:
    ModbusSensorReferenceNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          sensor_client_(io_) {
        loadConfig();
        pub_ = nh_.advertise<EquipmentState>(output_topic_, 10, false);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, rate_hz_)),
                                 &ModbusSensorReferenceNode::timerCallback,
                                 this);
        ROS_INFO_STREAM("modbus_sensor_reference_node publishing "
                        << output_topic_ << ", ins="
                        << (ins_enabled_ ? "enabled" : "disabled")
                        << ", mmwave="
                        << (radar_enabled_ ? "enabled" : "disabled")
                        << ", endpoint=" << device_.host << ":"
                        << device_.port << ", unit_id="
                        << device_.unit_id);
    }

private:
    void loadRegister(const std::string& ns, RegisterConfig& config) {
        private_nh_.param(ns + "/address", config.address, config.address);
        private_nh_.param(ns + "/scale", config.scale, config.scale);
    }

    void loadConfig() {
        private_nh_.param("output_topic", output_topic_, output_topic_);
        private_nh_.param("frame_id", frame_id_, frame_id_);
        private_nh_.param("rate_hz", rate_hz_, rate_hz_);
        private_nh_.param("host", device_.host, device_.host);
        private_nh_.param("port", device_.port, device_.port);
        private_nh_.param("unit_id", device_.unit_id, device_.unit_id);
        private_nh_.param("timeout_ms", device_.timeout_ms,
                          device_.timeout_ms);
        private_nh_.param("ins/enabled", ins_enabled_, ins_enabled_);
        private_nh_.param("mmwave/enabled", radar_enabled_, radar_enabled_);
        loadRegister("ins/roll", roll_);
        loadRegister("ins/pitch", pitch_);
        loadRegister("ins/yaw", yaw_);
        loadRegister("mmwave/left_front", left_front_);
        loadRegister("mmwave/left_rear", left_rear_);
        loadRegister("mmwave/right_front", right_front_);
        loadRegister("mmwave/right_rear", right_rear_);
    }

    double readScaled(const std::vector<std::uint16_t>& regs,
                      int base,
                      const RegisterConfig& config) const {
        const int index = config.address - base;
        if (index < 0 || index >= static_cast<int>(regs.size())) {
            return 0.0;
        }
        return static_cast<double>(signedRegister(regs[index])) * config.scale;
    }

    bool readGroup(const ModbusDeviceConfig& device,
                   ModbusClient& client,
                   const std::vector<RegisterConfig>& configs,
                   std::vector<std::uint16_t>& regs,
                   int& base) {
        int min_addr = configs.front().address;
        int max_addr = configs.front().address;
        for (const auto& config : configs) {
            min_addr = std::min(min_addr, config.address);
            max_addr = std::max(max_addr, config.address);
        }
        base = min_addr;
        return client.readHoldingRegisters(device, min_addr,
                                           max_addr - min_addr + 1, regs);
    }

    void timerCallback(const ros::TimerEvent&) {
        EquipmentState state;
        state.header.stamp = ros::Time::now();
        state.header.frame_id = frame_id_;
        state.source = "modbus_tcp";
        state.quality = "LOST";

        bool any_valid = false;
        bool read_failed = false;
        if (ins_enabled_) {
            std::vector<std::uint16_t> regs;
            int base = 0;
            if (readGroup(device_, sensor_client_, {roll_, pitch_, yaw_}, regs, base)) {
                state.roll_deg = readScaled(regs, base, roll_);
                state.pitch_deg = readScaled(regs, base, pitch_);
                state.yaw_deg = readScaled(regs, base, yaw_);
                state.attitude_valid = true;
                state.roll_valid = true;
                state.pitch_valid = true;
                state.yaw_valid = true;
                any_valid = true;
            } else {
                read_failed = true;
            }
        }
        if (radar_enabled_) {
            std::vector<std::uint16_t> regs;
            int base = 0;
            if (readGroup(device_, sensor_client_,
                          {left_front_, left_rear_, right_front_, right_rear_},
                          regs, base)) {
                state.left_front_mm = readScaled(regs, base, left_front_);
                state.left_rear_mm = readScaled(regs, base, left_rear_);
                state.right_front_mm = readScaled(regs, base, right_front_);
                state.right_rear_mm = readScaled(regs, base, right_rear_);
                state.left_front_clearance_m = state.left_front_mm / 1000.0;
                state.left_rear_clearance_m = state.left_rear_mm / 1000.0;
                state.right_front_clearance_m = state.right_front_mm / 1000.0;
                state.right_rear_clearance_m = state.right_rear_mm / 1000.0;
                state.distances_valid = true;
                state.left_front_valid = true;
                state.left_rear_valid = true;
                state.right_front_valid = true;
                state.right_rear_valid = true;
                any_valid = true;
            } else {
                read_failed = true;
            }
        }
        if (read_failed) {
            sensor_client_.close();
        }
        state.quality = any_valid ? "OK" : "LOST";
        state.invalid_reason = any_valid ? "none" : "TCP_READ_FAILED";
        pub_.publish(state);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    boost::asio::io_context io_;
    ModbusClient sensor_client_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::string output_topic_ = "/sensor_reference";
    std::string frame_id_ = "base_link";
    double rate_hz_ = 10.0;
    ModbusDeviceConfig device_;
    bool ins_enabled_ = false;
    bool radar_enabled_ = false;
    RegisterConfig roll_{0, 0.01};
    RegisterConfig pitch_{1, 0.01};
    RegisterConfig yaw_{2, 0.01};
    RegisterConfig left_front_{10, 1.0};
    RegisterConfig left_rear_{11, 1.0};
    RegisterConfig right_front_{12, 1.0};
    RegisterConfig right_rear_{13, 1.0};
};

}  // namespace
}  // namespace target_localizer

int main(int argc, char** argv) {
    ros::init(argc, argv, "modbus_sensor_reference_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    target_localizer::ModbusSensorReferenceNode node(nh, private_nh);
    ros::spin();
    return 0;
}
