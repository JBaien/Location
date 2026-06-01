#ifndef lidar_pointcloud_CALIBRATION_H
#define lidar_pointcloud_CALIBRATION_H

#include <map>
#include <vector>
#include <string>

namespace lidar_pointcloud
{

/** \brief Correction information for a single laser. */
struct LaserCorrection
{
  /** parameters in db.xml */
  float rot_correction;
  float vert_correction;
  float dist_correction;
  bool two_pt_correction_available;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;

  /** cached values calculated when the calibration file is read */
  float cos_rot_correction;              ///< cosine of rot_correction
  float sin_rot_correction;              ///< sine of rot_correction
  float cos_vert_correction;             ///< cosine of vert_correction
  float sin_vert_correction;             ///< sine of vert_correction

  int laser_ring;                        ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration
{
public:
  float distance_resolution_m;
  std::map<int, LaserCorrection> laser_corrections_map;
  std::vector<LaserCorrection> laser_corrections;
  int num_lasers;
  bool initialized;
  bool ros_info;

public:
  explicit Calibration(bool info = true) : distance_resolution_m(0.002f),
    num_lasers(0),
    initialized(false),
    ros_info(info) {}
  explicit Calibration(
    const std::string& calibration_file,
    bool info = true)
  : distance_resolution_m(0.002f),
    ros_info(info)
  {
    read(calibration_file);
  }

  void read(const std::string& calibration_file);
  void write(const std::string& calibration_file);
};

}  // namespace lidar_pointcloud

#endif  // lidar_pointcloud_CALIBRATION_H
