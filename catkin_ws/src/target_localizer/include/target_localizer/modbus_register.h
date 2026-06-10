#pragma once

#include <cstdint>

namespace target_localizer {

struct RegisterConfig {
    int address = 0;
    double scale = 1.0;
    bool is_signed = true;
};

inline double rawRegisterValue(std::uint16_t value, bool is_signed) {
    if (is_signed) {
        return static_cast<double>(static_cast<std::int16_t>(value));
    }
    return static_cast<double>(value);
}

inline double scaleRegisterValue(std::uint16_t value,
                                 const RegisterConfig& config) {
    return rawRegisterValue(value, config.is_signed) * config.scale;
}

}  // namespace target_localizer
