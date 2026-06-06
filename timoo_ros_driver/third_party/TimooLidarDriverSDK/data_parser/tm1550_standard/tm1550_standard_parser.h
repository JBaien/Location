
#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <time.h>
#include <iostream>


#include "interface/base_parser.h"
#include "interface/base_time.h"
#include "common/common.h"
#include "common/device_config.h"

namespace timoo {
namespace driver {


class TM1550StandardDataParser : public BaseParser {

public:
    TM1550StandardDataParser() = default;
    ~TM1550StandardDataParser() = default;
    int Init(const base::DeviceConfig& config) override;
    int parse(const RawDataPtr raw_data) override;


private:
    // Packet data parse, points data or gps data etc.
    int packetDataParse(const RawDataPtr raw_data);

    // // Status data parse, device info
    // int statusDataParse(const RawDataPtr raw_data);

    // // Points collect
    // void correctTimestamp(const base::Time& cur_pkt_time);
    // void collectPoints(const std::shared_ptr<PackageData>& package, const BlockData& block, 
    //                    const double& block_start_time);

    // Set frame info, start
    void setFrameStartInfo(const double& timestamp);

    // Set frame info, end
    void setFrameEndInfo(const double& timestamp);

    // Ratio coefficients
    const float DISTANCE_RATIO = 0.01;

};


} // namespace driver
} // namespace timoo




