#pragma once


#include <cstdint>
#include <limits>
#include <memory>
#include <vector>


namespace timoo {
namespace driver {
namespace base {

struct TimooPoint {

    float x = 0;
    float y = 0;
    float z = 0;
    std::uint8_t intensity = 0;
    float distance = 0;
    float angle = 0;
    double time = 0;
    int azimuth = 0;
    int ring_id = 0;

    bool valid = false;
};

struct IMUData{
    uint32_t time;
    float Accel_x;
    float Accel_y;
    float Accel_z;
    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    float temperature;
};


struct TimooPacketData {

    // common
    double timestamp = 0.0;
    int seq = 0;

    //imu
    bool is_imu = false;
    IMUData imu_data;

    //pointcloud
    bool is_last_packet = false;
    int cut_point_index = 0.0;
    double cut_point_time = 0.0;
    std::vector<TimooPoint> points;

};
using TimooPacketDataPtr = std::shared_ptr<TimooPacketData>;

struct TimooPointScan {

    double timestamp = 0;
    // TODO: Scan数据结构需定义

};
using TimooPointScanPtr = std::shared_ptr<TimooPointScan>;

struct TimooPointCloud {

    double timestamp = 0;
    std::vector<TimooPoint> points;

};
using TimooPointCloudPtr = std::shared_ptr<TimooPointCloud>;
using TimooIMUPtr = std::shared_ptr<IMUData>;


} // namespace base
} // namespace driver
} // namespace timoo
