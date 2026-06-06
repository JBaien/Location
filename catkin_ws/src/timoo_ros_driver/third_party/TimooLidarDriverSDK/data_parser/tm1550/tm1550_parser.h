
#pragma once

#include <vector>
#include <memory>
#include <time.h>
#include <iostream>


#include "../../interface/base_parser.h"
#include "../../common/common.h"
#include "../../common/device_config.h"

namespace timoo {
namespace driver {


class TM1550DataParser : public BaseParser {

public:
    TM1550DataParser() = default;
    ~TM1550DataParser() = default;
    int Init(const base::DeviceConfig& config) override;
    int parse(const RawDataPtr raw_data) override;


private:

};


} // namespace driver
} // namespace timoo




