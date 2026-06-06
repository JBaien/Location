
#pragma once

#include <memory>

#include "interface/base_input.h"


namespace timoo {
namespace driver {


/// ////////////////////////////
/// InputFactory
/// ////////////////////////////

class InputFactory
{
public:
    static BaseInputPtr MakeInputPCAP(const std::string& path, const double time_period);
    static BaseInputPtr MakeInputUDP(const std::string ip , const int port, const size_t buffer_size);
};


} // namespace driver
} // namespace timoo
