#pragma once

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace target_localizer {

struct EquipmentGeometryConfig {
    double ground_x_min = -4.0;
    double ground_x_max = 4.0;
    double ground_y_min = -2.5;
    double ground_y_max = 2.5;
    double ground_z_min = -2.0;
    double ground_z_max = 0.5;
    double wall_x_min = -6.0;
    double wall_x_max = 6.0;
    double wall_y_abs_min = 0.8;
    double wall_y_abs_max = 4.0;
    double wall_z_min = -1.0;
    double wall_z_max = 2.5;
    double front_sample_distance_m = 2.0;
    double rear_sample_distance_m = 2.0;
    double sample_window_x_m = 0.4;
    double side_y_abs_min = 0.2;
    double side_y_abs_max = 5.0;
    double side_z_min = -1.5;
    double side_z_max = 1.5;
    double distance_percentile = 0.10;
    int min_ground_points = 20;
    int min_wall_points = 20;
    int min_distance_points = 5;
    int forward_sign = 1;
    int left_sign = 1;
};

struct EquipmentGeometryResult {
    bool attitude_valid = false;
    bool distances_valid = false;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double left_front_mm = 0.0;
    double left_rear_mm = 0.0;
    double right_front_mm = 0.0;
    double right_rear_mm = 0.0;
    int ground_points = 0;
    int wall_points = 0;
    int left_front_points = 0;
    int left_rear_points = 0;
    int right_front_points = 0;
    int right_rear_points = 0;
    std::string quality = "lost";
};

EquipmentGeometryResult estimateEquipmentGeometry(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const EquipmentGeometryConfig& config);

}  // namespace target_localizer
