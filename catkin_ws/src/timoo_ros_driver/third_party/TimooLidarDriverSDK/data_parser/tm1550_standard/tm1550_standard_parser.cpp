
#include "tm1550_standard_parser.h"
#include "common/point_type.h"

#include <cmath>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

namespace timoo {
namespace driver {


int TM1550StandardDataParser::Init(const base::DeviceConfig& config) {

    cfg_ = config;

    return 0;
}
void TM1550StandardDataParser::setFrameStartInfo(const double& timestamp)
{
    pkt_data_ = std::make_shared<base::TimooPacketData>();
    pkt_data_->points.reserve(500);
    pkt_data_->is_last_packet = false;
    pkt_data_->timestamp = timestamp;
}

void TM1550StandardDataParser::setFrameEndInfo(const double& timestamp) 
{
    if (pkt_data_->points.empty()) { 

        // Trick, last udp packet is empty, ignore it, only deliver the last packet signal
        pkt_data_->points.emplace_back(base::TimooPoint());
    }
    pkt_data_->is_last_packet = true;
    pkt_data_->cut_point_index = pkt_data_->points.size();    
    pkt_data_->cut_point_time = timestamp;
}

int TM1550StandardDataParser::packetDataParse(const RawDataPtr raw_data) {

    // Set frame start info
    setFrameStartInfo(raw_data->time.toSec());

    // Data parse
    if(raw_data->d[0] == 0x6d) {

        for(int i = 5 ; i < 1349; i+=8)
        {
            base::TimooPoint pointtmep;

            pointtmep.x = float((raw_data->d[i+2] << 8) | (uint8_t)raw_data->d[i+1]) * DISTANCE_RATIO;
            pointtmep.y = float((raw_data->d[i+4] << 8) | (uint8_t)raw_data->d[i+3]) * DISTANCE_RATIO;
            pointtmep.z = float((raw_data->d[i+6] << 8) | (uint8_t)raw_data->d[i+5]) * DISTANCE_RATIO;
            pointtmep.intensity = uint8_t(raw_data->d[i + 7]);

            // Symbol bit
            u_char bytes = raw_data->d[i];
            int bitX = (bytes >> 2) & 1;
            int bitY = (bytes >> 1) & 1; 
            int bitZ = (bytes >> 0) & 1; 
            if(bitY == 1) {
                pointtmep.y = -pointtmep.y;
            }
            if(bitZ == 1) {
                pointtmep.z = -pointtmep.z;
            }

            // Noise filt
            if(pointtmep.x <= 0.1) {
                continue;
            }
            pkt_data_->points.push_back(pointtmep);
        }
    }

    // A completed frame data collected 
    if(raw_data->d[4] == 0x01) {
        setFrameEndInfo(raw_data->time.toSec());
    }

    return 0;
}


int TM1550StandardDataParser::parse(const RawDataPtr raw_data) {
    if (raw_data->type == DataType::STATUS) {
        return 0;
    } else if (raw_data->type == DataType::PACKET) {
        return packetDataParse(raw_data);
    } else {
        return -1;
    }

    return 0;
}



} // namespace driver
} // namespace timoo

