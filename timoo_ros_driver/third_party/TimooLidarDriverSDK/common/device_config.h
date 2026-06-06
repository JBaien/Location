#pragma once 

#include <cstdint>
#include <iostream>
#include <string>

#include "common.h"

namespace timoo {
namespace driver {
namespace base {



struct DeviceConfig
{

    SensorType lidar_type = SensorType::TIMOO16;

    std::string file_name="";
    bool read_fast = false;
    bool read_once = false;
    bool organize_cloud = true;

    bool use_tail_time = true;
    bool use_gps_clock = false;
    bool use_imu_data = false;

    float cut_angle = -1; // degree, [0.0, 360.0];  if -1, no cut
    std::string calibration_file = "";

    std::string host_ip = "";
    int udp_port = 2368;
    int status_port = 8603;

    int imu_src_port = 7788;
    int imu_dst_port = 7788;

    float max_distance = 150.0;
    float min_distance = 0.2;
    float max_angle = 360.0;
    float min_angle = 0.0;

    bool fixed_points_count = false;
};

} // namespace base
} // namespace driver
} // namespace timoo
