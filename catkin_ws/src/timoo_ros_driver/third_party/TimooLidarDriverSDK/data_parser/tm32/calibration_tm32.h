
#include <map>
#include <vector>
#include <string>

namespace timoo_pointcloud {
namespace tm32{
  
  /** \brief Correction information for a single laser. */
  struct LaserCorrection {

    /** parameters in db.xml */
    float rot_correction;
    float vert_correction;
    float dist_correction;
    bool two_pt_correction_available = false;
    float dist_correction_x = 0.0;
    float dist_correction_y = 0.0;
    float vert_offset_correction = 0.0;
    float horiz_offset_correction = 0.0;
    int max_intensity = 255;
    int min_intensity = 0;
    float focal_distance = 0.0;
    float focal_slope = 0.0;

    /** cached values calculated when the calibration file is read */
    float cos_rot_correction = 1;              ///< cosine of rot_correction
    float sin_rot_correction = 0;              ///< sine of rot_correction
    float cos_vert_correction;             ///< cosine of vert_correction
    float sin_vert_correction;             ///< sine of vert_correction

    int laser_ring;                        ///< ring number for this laser
  };

  /** \brief Calibration information for the entire device. */
  class Calibration {

  public:

    float distance_resolution_m;
    std::vector<LaserCorrection> laser_corrections;
    int num_lasers;

  };
  
} /* timoo_pointcloud */



}


