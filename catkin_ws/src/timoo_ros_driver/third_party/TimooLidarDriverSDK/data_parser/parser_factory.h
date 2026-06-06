
#pragma once

#include <vector>
#include <memory>
#include <time.h>
#include <iostream>


#include "../interface/base_parser.h"
#include "../common/common.h"


namespace timoo {
namespace driver {


/// ////////////////////////////
/// ParserFactory
/// ////////////////////////////

class ParserFactory
{
public:
    static BaseParserPtr MakeDataParser(const base::SensorType& sensor_type);
    static BaseParserPtr MakeDataParser(const std::string& sensor_type);
};


} // namespace driver
} // namespace timoo




