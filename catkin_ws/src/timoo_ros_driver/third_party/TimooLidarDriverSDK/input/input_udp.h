#pragma once

#include "interface/base_input.h"

#include <cstddef>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <vector>
#include <arpa/inet.h>  // 提供 inet_addr()

namespace timoo {
namespace driver {


class InputUDP : public BaseInput
{
public:
    explicit InputUDP();
    ~InputUDP() override;

    bool Init(const std::string ip ,const int port, const size_t buffer_size);

    int GetPackage(RawData* data) override;

private:
    int sockfd_;
    in_addr devip_;

    size_t buffer_size_;

    int port_;
    
    int socket_ = -1;
    struct sockaddr_in addr_;

    // Data pointer
    std::vector<u_char> data_;
};

} // namespace driver
} // namespace timoo
