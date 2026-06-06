
#include "parser_factory.h"

#include "./tm16/tm16_parser.h"
#include "./tm32/tm32_parser.h"
#include "./tm1550_standard/tm1550_standard_parser.h"
#include "./tm128/tm128_parser.h"
#include "./tm128v/tm128v_parser.h"
namespace timoo {
namespace driver {

BaseParserPtr ParserFactory::MakeDataParser(const base::SensorType& sensor_type) {

    if (sensor_type == base::SensorType::TIMOO16) {
        auto parser = std::make_shared<TM16DataParser>();
        // if (!parser->Init()) {
        //     return nullptr;
        // }
        return parser;
    }  else if (sensor_type == base::SensorType::TIMOO32) {
        auto parser = std::make_shared<tm32::TM32DataParser>();
        // if (!parser->Init()) {
        //     return nullptr;
        // }
        return parser;
    } else if (sensor_type == base::SensorType::TIMOO1550STD) {
        auto parser = std::make_shared<TM1550StandardDataParser>();
        // if (!parser->Init()) {
        //     return nullptr;
        // }
        return parser;
    } else if (sensor_type == base::SensorType::TIMOO128) {
        auto parser = std::make_shared<TM128DataParser>();
        // if (!parser->Init()) {
        //     return nullptr;
        // }
        return parser;
    } else if (sensor_type == base::SensorType::TIMOO128V) {
        auto parser = std::make_shared<TM128VDataParser>();
        // if (!parser->Init()) {
        //     return nullptr;
        // }
        return parser;
    }

    return nullptr;

}

BaseParserPtr ParserFactory::MakeDataParser(const std::string& sensor_type) {

    return nullptr;
}


} // namespace driver
} // namespace timoo



