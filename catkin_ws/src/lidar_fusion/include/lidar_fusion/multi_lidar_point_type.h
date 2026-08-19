#pragma once

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace lidar_fusion {

// Fused point type. lidar_id keeps the source sensor separate, ring identifies
// the physical vertical beam, and azimuth/range retain the source-sensor polar
// topology before TF moves XYZ into base_link. The browser must use these
// source-local fields instead of inferring scan columns from transformed XYZ or
// a float timestamp.
struct EIGEN_ALIGN16 PointXYZIRTL {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    float time = 0.0f;
    float azimuth = 0.0f;  // radians in [0, 2*pi), source lidar frame
    float range = 0.0f;    // metres from the source lidar origin
    std::uint16_t ring = 0;
    std::uint8_t lidar_id = 0;
    std::uint8_t _padding = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace lidar_fusion

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_fusion::PointXYZIRTL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (float, azimuth, azimuth)
    (float, range, range)
    (std::uint16_t, ring, ring)
    (std::uint8_t, lidar_id, lidar_id))
