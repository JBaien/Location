
#pragma once 

#include <cstdint>
#include <iostream>
#include <string>
#include <map>


namespace timoo {
namespace driver {
namespace base {


enum class SensorType : std::uint8_t {
    UNKNOWN = 0,
    TIMOO16 = 1,
    TIMOO32 = 2,
    TIMOO1550STD = 3,
    TIMOO1550 = 4,
    TIMOO128 = 5,
    TIMOO128V = 6
};


static std::map<std::string, SensorType> lidar_type_map {
    {"timoo_16", SensorType::TIMOO16},
    {"timoo_32", SensorType::TIMOO32},
    {"timoo_1550_standard", SensorType::TIMOO1550STD},
    {"timoo_1550", SensorType::TIMOO1550},
    {"timoo_128", SensorType::TIMOO128},
    {"timoo_128v", SensorType::TIMOO128V},
};


} // namespace base
} // namespace driver
} // namespace timoo
