
#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <atomic>
#include <time.h>
#include <iostream>


#include "interface/base_parser.h"
#include "interface/base_time.h"
#include "common/common.h"
#include "common/device_config.h"
#include "calibration.h"

namespace timoo {
namespace driver {



/** Special Defines for TM16 support **/
static const int TM16_FIRINGS_PER_BLOCK = 2;
static const int TM16_SCANS_PER_FIRING = 16;
static const float TM16_BLOCK_TDURATION = 98.304f;  // [µs]
static const float TM16_DSR_TOFFSET = 3.072f;        // [µs]
static const float TM16_FIRING_TOFFSET = 49.152f;    // [µs]

/**
 * Raw timoo packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
static const float ROTATION_RESOLUTION = 0.01f;  // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

// TM16 channel order
static const std::vector<int> TM16_CHANNEL_MAP {
    0, 2, 4, 6, 8, 10, 12, 14, 
    1, 3, 5, 7, 9, 11, 13, 15 
};

// TM16 channel to angle value
static const std::vector<float> TM16_CHANNEL_TO_ANGLE_VALUE {
    -15,1,-13,3,-11,5,-9,7,
    -7,9,-5,11,-3,13,-1,15
};

static const double FULL_FIRING_CYCLE = 49.152 * 1e-3; // milli seconds
static const double SINGLE_FIRING = 3.072 * 1e-3;      // milli seconds

static const uint32_t FULL_FIRING_CYCLE_NANO = 49152; // nano seconds
static const uint32_t SINGLE_FIRING_NANO = 3072;      // nano seconds


/// ///////////////////////////////
/// Package Data
/// ///////////////////////////////

// Float4Byte, status packet data struct
union Float4Byte{
    uint8_t bytes[4];
    float value;
};

// UnitData
struct UnitData
{
    float dist;
    float intensity;
};

// BlockData
class BlockData
{
public:
    void Load(const unsigned char* buf, int unit_num);

private:
    float angle_;
    float block_duration_;
    std::vector<UnitData> units_;

public:
    const float& angle() const { return angle_; };
    const float& block_duration() const { return block_duration_; };
    void set_angle(float ang) { this->angle_ = ang; };
    void set_block_duration(float ang) { this->block_duration_ = ang; };
    const std::vector<UnitData>& units() const { return units_; };
};

typedef std::shared_ptr<BlockData> BlockDataPtr;

// PackageData
class PackageData
{
public:
    // void Load(const unsigned char* data, 
    // int block_offset, int block_size, int block_num, int unit_num);
    void Load(const unsigned char* buf);

private:
    std::vector<BlockDataPtr> blocks_;

    base::Time unix_sec_; // lidar interial data time

    base::Time sys_sec_;

public:
    void set_sys_sec(base::Time val) { sys_sec_ = val; };
    void set_unix_sec(base::Time val) { unix_sec_ = val; };

    base::Time unix_sec() const { return unix_sec_; };
    base::Time sys_sec() const { return sys_sec_; };

    const std::vector<BlockDataPtr>& blocks() const { return blocks_; };
};

typedef std::shared_ptr<PackageData> PackageDataPtr;



class TM16DataParser : public BaseParser {

public:
    TM16DataParser() = default;
    ~TM16DataParser() = default;
    int Init(const base::DeviceConfig& config) override;
    int parse(const RawDataPtr raw_data) override;


private:
    // Packet data parse, points data or gps data etc.
    int packetDataParse(const RawDataPtr raw_data);

    // Status data parse, device info
    int statusDataParse(const RawDataPtr raw_data);

    // Points collect
    void correctTimestamp(const base::Time& cur_pkt_time);
    void collectPoints(const std::shared_ptr<PackageData>& package, const BlockData& block, 
                       const double& block_start_time);

    // Set frame info, start
    void setFrameStartInfo(const double& timestamp);

    // Check GPS status, from DIFOP packet
    void checkGPSStatus(const RawDataPtr raw_data);

    // Extract packet time, GPS or NTP time
    base::Time extractPacketTime(const RawDataPtr raw_data);

    // Set frame info, end
    void setFrameEndInfo(const double& timestamp);

    // Print function. vertical calibration warning
    void printVerticalCalibWarning();

    // 记录上次打印垂直校准警告的时间点
    std::chrono::steady_clock::time_point last_vertical_calib_warning_time_;

    /** 
     * Calibration file
     */
    timoo_pointcloud::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    int imuparse(const RawDataPtr raw_data);
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

    /** 
     *  Time compensation
     */
    double pkt_duration_;
    double block_duration_;
    double single_point_duration_;

    // GPS status mark
    std::atomic<bool> gps_status_;

};


} // namespace driver
} // namespace timoo




