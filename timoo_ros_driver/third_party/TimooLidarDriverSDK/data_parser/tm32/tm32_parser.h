
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
#include "calibration_tm32.h"


namespace timoo {
namespace driver {
namespace tm32 {
/**
 * Raw timoo packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for TM16 support **/
static  const int TM32_SCANS_PER_FIRING =  32;
static const float TM32_BLOCK_TDURATION = 1.736 * 32 * 1e-3;//  // milli seconds
static const double TM32_SINGLE_FIRING = 1.736 * 1e-3;  //?    // milli seconds

/** \brief Raw timoo data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);


// TM32 channel order
static const std::vector<int> TM32_CHANNEL_MAP {
    0, 1, 8, 9, 16, 17, 24, 25, 2, 3, 10, 11, 18, 19, 26, 27, 
    4, 5, 12, 13, 20, 21, 28, 29, 6, 7, 14, 15, 22, 23, 30, 31 
};


// TM32 channel to angle value
static const std::vector<float> TM32_CHANNEL_TO_ANGLE_VALUE{
	-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, 
    -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
};



/** \brief Raw timoo packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  //uint16_t revolution;   ------------
  //uint8_t status[PACKET_STATUS_SIZE];   -------------
  uint32_t timestamp;
  uint16_t factory;

}
raw_packet_t;



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



class TM32DataParser : public BaseParser {

public:
    TM32DataParser() = default;
    ~TM32DataParser() = default;
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

    // Set frame info, end
    void setFrameEndInfo(const double& timestamp);

    // Check GPS status, from DIFOP packet
    void checkGPSStatus(const RawDataPtr raw_data);

    // Extract packet time, GPS or NTP time
    base::Time extractPacketTime(const RawDataPtr raw_data);

    // Print function. vertical calibration warning
    void printVerticalCalibWarning();

    // 记录上次打印垂直校准警告的时间点
    std::chrono::steady_clock::time_point last_vertical_calib_warning_time_;

    /** 
     * Calibration file
     */
    timoo_pointcloud::tm32::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    /** 
     * Vertical angel correction table
     */
    std::vector<float> vertical_angle_list_;
    bool calibrate_updated_ = false;

    /** 
     * Azimuth angel between two planes
     */
    int16_t azimuth_diff_two_planes_ = 0;  
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
}

}
}