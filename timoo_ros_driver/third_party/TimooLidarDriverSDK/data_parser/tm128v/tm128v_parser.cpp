
#include "tm128v_parser.h"

#include <cmath>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string.h>
#include <unistd.h>

namespace timoo {
namespace driver {

static inline double get_current_time_in_seconds() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}


inline void setNaN(base::TimooPoint& point)
{
    point.x = nanf("");
    point.y = nanf("");
    point.z = nanf("");
    point.intensity = 0;  // Use 0 to represent invalid or undefined intensity;
}

inline float SQR(const float& val) { return val*val; }


int TM128VDataParser::Init(const base::DeviceConfig& config) {

    cfg_ = config;

    std::cout << "[TM128VDataParser] Init!" << std::endl;
    return 0;
}


void TM128VDataParser::setFrameStartInfo(const double& timestamp)
{
    pkt_data_ = std::make_shared<base::TimooPacketData>();
    pkt_data_->points.reserve(33800);
    pkt_data_->is_last_packet = false;
    pkt_data_->timestamp = timestamp;
}

void TM128VDataParser::setFrameEndInfo(const double& timestamp) 
{
    // if (pkt_data_->points.empty()) { return; }
    pkt_data_->is_last_packet = true;
    pkt_data_->cut_point_index = pkt_data_->points.size();    
    pkt_data_->cut_point_time = timestamp;
}


int TM128VDataParser::packetDataParse(const RawDataPtr raw_data)
{

    if (raw_data->size < 841) { return -2; }

    // Set frame start info
    setFrameStartInfo(raw_data->time.toSec());

    // Parse package data
    std::vector<u_char> data = raw_data->d;
    customer_analysis_.analysis(data, pkt_data_->points, pkt_data_->is_last_packet);

    // std::cout << "pkt_data_->points size:" << pkt_data_->points.size() << " ,  is_last_packet:" << pkt_data_->is_last_packet << std::endl;

    if (pkt_data_->is_last_packet) {
        setFrameEndInfo(pkt_data_->points.back().time);
    }

    return 0;
}



int TM128VDataParser::statusDataParse(const RawDataPtr raw_data)
{
    return 0;
}

int TM128VDataParser::parse(const RawDataPtr raw_data) {
    if (raw_data->type == DataType::STATUS) {
        return statusDataParse(raw_data);
    } else if (raw_data->type == DataType::PACKET) {
        return packetDataParse(raw_data);
    } else {
        return -1;
    }

    return 0;
}



} // namespace driver
} // namespace timoo

