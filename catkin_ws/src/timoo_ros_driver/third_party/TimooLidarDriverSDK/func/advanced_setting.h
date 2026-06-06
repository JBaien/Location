#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <memory>


namespace timoo {
namespace driver {


class AdvancedSetting {
public:
    // Constructor
    AdvancedSetting(int port);

    // Destructor
    ~AdvancedSetting();

    // 更新数据
    void updateBuffer(const std::vector<u_char>& data);

private:

    int port_ = 8603;
    std::string ip_addr_;

    std::vector<u_char > buf_;
    std::mutex buf_mutex_;
    std::vector<u_char > first_packet_buf_;
    bool checkpacket(const std::vector<u_char>& data);

public:
    bool setDormant();
    bool setWakeUp();
    void getStatus() const; // 获取状态信息，只有休眠和唤醒两种状态，1：休眠  0：唤醒
    const std::string& getDeviceInfo() const {return ip_addr_; }
    bool SendFirstPacket();
};


using AdvancedSettingPtr = std::shared_ptr<AdvancedSetting>;

} // namespace driver
} // namespace timoo










