

#include <memory>

#include "input_factory.h"

#include "input_pcap.h"
#include "input_udp.h"

namespace timoo {
namespace driver {


/// ////////////////////////////
/// InputFactory
/// ////////////////////////////


BaseInputPtr InputFactory::MakeInputPCAP(const std::string& path, const double time_period) {
    auto input = std::make_shared<InputPCAP>();
    if (!input->Init(path, time_period)) {
        return nullptr;
    }

    return input;
}


BaseInputPtr InputFactory::MakeInputUDP(const std::string ip ,const int port, const size_t buffer_size) {
    auto input = std::make_shared<InputUDP>();
    if (!input->Init(ip,port, buffer_size)) {
        return nullptr;
    }

    return input;
}


} // namespace driver
} // namespace timoo
