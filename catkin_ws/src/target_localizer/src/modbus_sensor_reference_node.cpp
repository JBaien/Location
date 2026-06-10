#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <target_localizer/EquipmentState.h>

#include "target_localizer/modbus_register.h"

namespace target_localizer {
namespace {

using boost::asio::ip::tcp;

struct ModbusDeviceConfig {
    std::string host = "127.0.0.1";
    int port = 502;
    int unit_id = 1;
    int timeout_ms = 500;
};

class ModbusClient {
public:
    explicit ModbusClient(boost::asio::io_service& io) : io_(io), socket_(io) {}

    bool readHoldingRegisters(const ModbusDeviceConfig& config,
                              int start_address,
                              int count,
                              std::vector<std::uint16_t>& out) {
        out.clear();
        boost::system::error_code ec;
        if (!socket_.is_open()) {
            tcp::resolver resolver(io_);
            tcp::resolver::query query(config.host,
                                       std::to_string(config.port));
            const auto endpoints = resolver.resolve(query, ec);
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
    boost::asio::io_service& io_;
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
    template <typename T>
    void loadParam(const std::string& name, T& value) {
        const std::string nested_name = "modbus/" + name;
        if (private_nh_.hasParam(nested_name)) {
            private_nh_.param(nested_name, value, value);
            return;
        }
        private_nh_.param(name, value, value);
    }

    void loadRegister(const std::string& ns, RegisterConfig& config) {
        loadParam(ns + "/address", config.address);
        loadParam(ns + "/scale", config.scale);
        loadParam(ns + "/signed", config.is_signed);
    }

    void loadConfig() {
        loadParam("output_topic", output_topic_);
        loadParam("frame_id", frame_id_);
        loadParam("rate_hz", rate_hz_);
        loadParam("host", device_.host);
        loadParam("port", device_.port);
        loadParam("unit_id", device_.unit_id);
        loadParam("timeout_ms", device_.timeout_ms);
        loadParam("ins/enabled", ins_enabled_);
        loadParam("mmwave/enabled", radar_enabled_);
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
        return scaleRegisterValue(regs[index], config);
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

    static void setAttitudeQuality(EquipmentState& state,
                                   const std::string& quality,
                                   const std::string& reason) {
        state.roll_quality = quality;
        state.pitch_quality = quality;
        state.yaw_quality = quality;
        state.roll_invalid_reason = reason;
        state.pitch_invalid_reason = reason;
        state.yaw_invalid_reason = reason;
    }

    static void setDistanceQuality(EquipmentState& state,
                                   const std::string& quality,
                                   const std::string& reason) {
        state.left_front_quality = quality;
        state.left_rear_quality = quality;
        state.right_front_quality = quality;
        state.right_rear_quality = quality;
        state.left_front_invalid_reason = reason;
        state.left_rear_invalid_reason = reason;
        state.right_front_invalid_reason = reason;
        state.right_rear_invalid_reason = reason;
    }

    void timerCallback(const ros::TimerEvent&) {
        EquipmentState state;
        state.header.stamp = ros::Time::now();
        state.header.frame_id = frame_id_;
        state.source = "modbus_tcp";
        state.quality = "LOST";
        state.overall_status = "LOST";
        setAttitudeQuality(state, "INVALID", "SENSOR_DISABLED");
        setDistanceQuality(state, "INVALID", "SENSOR_DISABLED");

        bool ins_valid = false;
        bool radar_valid = false;
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
                setAttitudeQuality(state, "OK", "none");
                ins_valid = true;
            } else {
                setAttitudeQuality(state, "INVALID", "TCP_READ_FAILED");
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
                setDistanceQuality(state, "OK", "none");
                radar_valid = true;
            } else {
                setDistanceQuality(state, "INVALID", "TCP_READ_FAILED");
                read_failed = true;
            }
        }
        if (read_failed) {
            sensor_client_.close();
        }
        const bool any_enabled = ins_enabled_ || radar_enabled_;
        const bool any_valid = ins_valid || radar_valid;
        const bool all_enabled_valid =
            (!ins_enabled_ || ins_valid) && (!radar_enabled_ || radar_valid);
        if (!any_enabled) {
            state.overall_status = "LOST";
            state.invalid_reason = "SENSOR_DISABLED";
        } else if (all_enabled_valid) {
            state.overall_status = "OK";
            state.invalid_reason = "none";
        } else if (any_valid) {
            state.overall_status = "DEGRADED";
            state.invalid_reason = "TCP_READ_FAILED";
        } else {
            state.overall_status = "LOST";
            state.invalid_reason = "TCP_READ_FAILED";
        }
        state.quality = state.overall_status;
        pub_.publish(state);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    boost::asio::io_service io_;
    ModbusClient sensor_client_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::string output_topic_ = "/sensor_reference";
    std::string frame_id_ = "base_link";
    double rate_hz_ = 10.0;
    ModbusDeviceConfig device_;
    bool ins_enabled_ = false;
    bool radar_enabled_ = false;
    RegisterConfig roll_{0, 0.01, true};
    RegisterConfig pitch_{1, 0.01, true};
    RegisterConfig yaw_{2, 0.01, true};
    RegisterConfig left_front_{10, 1.0, false};
    RegisterConfig left_rear_{11, 1.0, false};
    RegisterConfig right_front_{12, 1.0, false};
    RegisterConfig right_rear_{13, 1.0, false};
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
