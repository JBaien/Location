
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
#include "CustomerAnalysisV1_132.h"

namespace timoo {
namespace driver {





class TM128VDataParser : public BaseParser {

public:
    TM128VDataParser() = default;
    ~TM128VDataParser() = default;
    int Init(const base::DeviceConfig& config) override;
    int parse(const RawDataPtr raw_data) override;


private:
    // Packet data parse, points data or gps data etc.
    int packetDataParse(const RawDataPtr raw_data);

    // Status data parse, device info
    int statusDataParse(const RawDataPtr raw_data);

    // Set frame info, start
    void setFrameStartInfo(const double& timestamp);

    // Set frame info, end
    void setFrameEndInfo(const double& timestamp);

    /** 
     * Vertical angel correction table
     */
    std::vector<float> vertical_angle_list_;
    bool vertical_calibrate_updated_ = false;

    /** 
     * Cut frame by angle
     */
    double last_angle_ = -1;

    /** 
     * Frame time info
     */
    bool is_first_pkt_ = true;
    base::Time last_time_;

	CustomerAnalysisV1_132 customer_analysis_;

 
};


} // namespace driver
} // namespace timoo




