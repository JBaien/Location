
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "base_time.h"


namespace timoo {
namespace driver {


enum class DataType : std::uint8_t {
    STATUS = 0,
    PACKET = 1,
    IMU = 2,
};

struct RawData
{
    DataType type = DataType::PACKET;
    // const u_char* data;
    std::vector<u_char> d;
    int size;
    base::Time time;

    RawData() = default;
    RawData(const DataType& _type)
        : type(_type) {}

};

typedef std::shared_ptr<RawData> RawDataPtr;

} // namespace driver
} // namespace timoo
