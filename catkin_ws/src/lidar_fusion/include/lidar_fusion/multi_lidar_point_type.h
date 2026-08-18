#pragma once

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace lidar_fusion {

// Fused point type. The extra lidar_id field identifies the source sensor while
// preserving the original ring and per-point time used by the SLAM frontend.
struct EIGEN_ALIGN16 PointXYZIRTL {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    std::uint16_t ring = 0;
    std::uint8_t lidar_id = 0;
    std::uint8_t _padding = 0;
    float time = 0.0f;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace lidar_fusion

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_fusion::PointXYZIRTL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (std::uint8_t, lidar_id, lidar_id)
    (float, time, time))
