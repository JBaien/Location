#pragma once

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace lidar_fusion {

// Fused point type. Keep the legacy x/y/z/intensity/ring/lidar_id/time memory
// layout unchanged, then append source-polar fields. Existing consumers that
// match the original fields by name or offset therefore remain compatible.
struct EIGEN_ALIGN16 PointXYZIRTL {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    std::uint16_t ring = 0;
    std::uint8_t lidar_id = 0;
    std::uint8_t _padding = 0;
    float time = 0.0f;
    float azimuth = 0.0f;  // radians in [0, 2*pi), source lidar frame
    float range = 0.0f;    // metres from the source lidar origin
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
    (float, time, time)
    (float, azimuth, azimuth)
    (float, range, range))
