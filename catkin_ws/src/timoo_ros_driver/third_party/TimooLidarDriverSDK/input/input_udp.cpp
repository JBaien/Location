#include "input_udp.h"

#include <iostream>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <chrono>


namespace timoo {
namespace driver {



InputUDP::InputUDP() {}

InputUDP::~InputUDP()
{
    if (socket_ >= 0) close(socket_);
}

bool InputUDP::Init(const std::string ip,const int port, const size_t buffer_size)
{   
    port_ = port;
    buffer_size_ = buffer_size;
    data_.resize(buffer_size_);
    // data_ = new u_char[buffer_size];

    // Close last socket
    if (socket_ >= 0) close(socket_);

    // Init new socket
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        std::cout << "[InputUDP::Init] Socket init error." << std::endl;
        return false;
    }

    // Init new address
    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);

    if (!ip.empty()) {
        if (inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr) <= 0) {
            perror("Invalid IP address");
            close(socket_);
            return false;
        }
    } else {
        // 如果IP为空，绑定到所有网络接口
        addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    }

    struct timeval timeout;
    timeout.tv_sec = 1;  // 1秒
    timeout.tv_usec = 0; // 0微秒

    // 设置接收超时
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt failed");
        close(socket_);
        return false;
    }

    // Bind socket and address
    if (bind(socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
    {
        close(socket_);
        std::cout << "[InputUDP::Init] Socket bind error. ip:" << ip << ", port:" << port << std::endl;
        return false; 
    } 

    return true;
}

int InputUDP::GetPackage(RawData* data) 
{
    if (socket_ < 0) return -1;

    int len = sizeof(addr_);
    ssize_t nbytes = recvfrom(socket_, data_.data(), buffer_size_, 0, (struct sockaddr*)&addr_, (socklen_t*)&len);
    if (nbytes < 0) 
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            std::cout << "[InputUDP::GetPackage] UDP reading error! time out! port:" << port_  << ", " << strerror(errno) << std::endl;
            return -3;
        } else {
            std::cout << "[InputUDP::GetPackage] UDP reading error! port:" << port_ << std::endl;
            return -1;
        }
    }
    else
    {
        // data->data = data_;
        data->d.resize(data_.size());
        std::copy(data_.begin(), data_.end(), data->d.begin());

        data->size = nbytes;
        data->time = base::Time::now();
    }

    return 0;
}

} // namespace driver
} // namespace timoo
