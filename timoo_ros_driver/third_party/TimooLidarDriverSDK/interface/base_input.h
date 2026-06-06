#pragma once
#include <memory>
#include <string>
#include "base_data.h"


namespace timoo {
namespace driver {


#define ETHERNET_HEADER_SIZE 42


/// ////////////////////////////
/// Input
/// ////////////////////////////

class BaseInput
{
public:
    BaseInput() = default;
    virtual ~BaseInput() = default;

    virtual int GetPackage(RawData* data) = 0;
};

typedef std::shared_ptr<BaseInput> BaseInputPtr;

} // namespace driver
} // namespace timoo
