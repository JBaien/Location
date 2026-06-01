#ifndef lidar_pointcloud_RAWDATA_H
#define lidar_pointcloud_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <lidar_msgs/lidarScan.h>
#include <lidar_pointcloud/calibration.h>
#include <lidar_pointcloud/datacontainerbase.h>

namespace lidar_rawdata
{
/**
 * Raw lidar packet constants and structures.
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

/** Special Defines for VLP16 support **/
static const int lidar_FIRINGS_PER_BLOCK = 2;
static const int lidar_SCANS_PER_FIRING = 16;
static const float lidar_BLOCK_TDURATION = 110.592f;  // [µs]
static const float lidar_DSR_TOFFSET = 2.304f;        // [µs]
static const float lidar_FIRING_TOFFSET = 55.296f;    // [µs]

/** \brief Raw lidar data block.
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

/** \brief Raw lidar packet.
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
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;

/** \brief lidar data conversion class */
class RawData
{
public:
  RawData();
  ~RawData()
  {
  }

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns an optional calibration
   */
  boost::optional<lidar_pointcloud::Calibration> setup(ros::NodeHandle private_nh);

  /** \brief Set up for data processing offline.
   * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
   * this method is useful if unpacking data directly from bag files, without passing
   * through a communication overhead.
   *
   * @param calibration_file path to the calibration file
   * @param max_range_ cutoff for maximum range
   * @param min_range_ cutoff for minimum range
   * @returns 0 if successful;
   *           errno value for failure
   */
  int setupOffline(std::string calibration_file, double max_range_, double min_range_);

  void unpack(const lidar_msgs::lidarPacket& pkt, DataContainerBase& data,int sn_packet,const ros::Time& scan_start_time);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int scansPerPacket() const;

private:
  /** configuration parameters */
  typedef struct
  {
    std::string calibrationFile;  ///< calibration file name
    double max_range;             ///< maximum range to publish
    double min_range;             ///< minimum range to publish
    int min_angle;                ///< minimum angle to publish
    int max_angle;                ///< maximum angle to publish

    double tmp_min_angle;
    double tmp_max_angle;
  }
  Config;
  Config config_;

  /**
   * Calibration file
   */
  lidar_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  /** add private function to handle the lidar **/
  void unpack_lidar(const lidar_msgs::lidarPacket& pkt, DataContainerBase& data, int sn_packet,const ros::Time& scan_start_time);
};

}  // namespace lidar_rawdata

#endif  // lidar_pointcloud_RAWDATA_H
