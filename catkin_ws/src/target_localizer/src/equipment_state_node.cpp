#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <target_localizer/EquipmentState.h>

#include "target_localizer/equipment_geometry.h"

namespace target_localizer {
namespace {

struct NodeConfig {
    std::string input_topic = "/points_raw";
    std::string output_topic = "/equipment_state";
    std::string target_frame = "base_link";
    std::string required_frame_id = "base_link";
    double max_pointcloud_age_sec = 0.5;
    double distance_filter_alpha = 0.3;
    double max_distance_jump_m = 0.3;
    EquipmentGeometryConfig geometry;
};

class EquipmentStateNode {
public:
    EquipmentStateNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh) {
        loadConfig();
        sub_ = nh_.subscribe(config_.input_topic, 2,
                             &EquipmentStateNode::cloudCallback, this);
        pub_ = nh_.advertise<EquipmentState>(config_.output_topic, 10, false);
        ROS_INFO_STREAM("equipment_state_node subscribed to "
                        << config_.input_topic << ", publishing "
                        << config_.output_topic);
    }

private:
    void loadConfig() {
        private_nh_.param("input_topic", config_.input_topic,
                          config_.input_topic);
        private_nh_.param("equipment_state_topic", config_.output_topic,
                          config_.output_topic);
        private_nh_.param("target_frame", config_.target_frame,
                          config_.target_frame);
        private_nh_.param("required_frame_id", config_.required_frame_id,
                          config_.required_frame_id);
        private_nh_.param("max_pointcloud_age_sec",
                          config_.max_pointcloud_age_sec,
                          config_.max_pointcloud_age_sec);
        private_nh_.param("min_total_points",
                          config_.geometry.min_total_points,
                          config_.geometry.min_total_points);
        private_nh_.param("distance_filter_alpha",
                          config_.distance_filter_alpha,
                          config_.distance_filter_alpha);
        private_nh_.param("max_distance_jump_m",
                          config_.max_distance_jump_m,
                          config_.max_distance_jump_m);
        private_nh_.param("ground_x_min", config_.geometry.ground_x_min,
                          config_.geometry.ground_x_min);
        private_nh_.param("ground_x_max", config_.geometry.ground_x_max,
                          config_.geometry.ground_x_max);
        private_nh_.param("ground_y_min", config_.geometry.ground_y_min,
                          config_.geometry.ground_y_min);
        private_nh_.param("ground_y_max", config_.geometry.ground_y_max,
                          config_.geometry.ground_y_max);
        private_nh_.param("ground_z_min", config_.geometry.ground_z_min,
                          config_.geometry.ground_z_min);
        private_nh_.param("ground_z_max", config_.geometry.ground_z_max,
                          config_.geometry.ground_z_max);
        private_nh_.param("wall_x_min", config_.geometry.wall_x_min,
                          config_.geometry.wall_x_min);
        private_nh_.param("wall_x_max", config_.geometry.wall_x_max,
                          config_.geometry.wall_x_max);
        private_nh_.param("wall_y_abs_min", config_.geometry.wall_y_abs_min,
                          config_.geometry.wall_y_abs_min);
        private_nh_.param("wall_y_abs_max", config_.geometry.wall_y_abs_max,
                          config_.geometry.wall_y_abs_max);
        private_nh_.param("wall_z_min", config_.geometry.wall_z_min,
                          config_.geometry.wall_z_min);
        private_nh_.param("wall_z_max", config_.geometry.wall_z_max,
                          config_.geometry.wall_z_max);
        private_nh_.param("front_sample_distance_m",
                          config_.geometry.front_sample_distance_m,
                          config_.geometry.front_sample_distance_m);
        private_nh_.param("rear_sample_distance_m",
                          config_.geometry.rear_sample_distance_m,
                          config_.geometry.rear_sample_distance_m);
        private_nh_.param("sample_window_x_m",
                          config_.geometry.sample_window_x_m,
                          config_.geometry.sample_window_x_m);
        private_nh_.param("side_y_abs_min", config_.geometry.side_y_abs_min,
                          config_.geometry.side_y_abs_min);
        private_nh_.param("side_y_abs_max", config_.geometry.side_y_abs_max,
                          config_.geometry.side_y_abs_max);
        private_nh_.param("side_z_min", config_.geometry.side_z_min,
                          config_.geometry.side_z_min);
        private_nh_.param("side_z_max", config_.geometry.side_z_max,
                          config_.geometry.side_z_max);
        private_nh_.param("distance_percentile",
                          config_.geometry.distance_percentile,
                          config_.geometry.distance_percentile);
        private_nh_.param("equipment_half_width_m",
                          config_.geometry.equipment_half_width_m,
                          config_.geometry.equipment_half_width_m);
        private_nh_.param("min_valid_clearance_m",
                          config_.geometry.min_valid_clearance_m,
                          config_.geometry.min_valid_clearance_m);
        private_nh_.param("max_ground_plane_rmse",
                          config_.geometry.max_ground_plane_rmse,
                          config_.geometry.max_ground_plane_rmse);
        private_nh_.param("min_ground_normal_z",
                          config_.geometry.min_ground_normal_z,
                          config_.geometry.min_ground_normal_z);
        private_nh_.param("max_wall_direction_diff_deg",
                          config_.geometry.max_wall_direction_diff_deg,
                          config_.geometry.max_wall_direction_diff_deg);
        private_nh_.param("min_ground_points",
                          config_.geometry.min_ground_points,
                          config_.geometry.min_ground_points);
        private_nh_.param("min_wall_points",
                          config_.geometry.min_wall_points,
                          config_.geometry.min_wall_points);
        private_nh_.param("min_distance_points",
                          config_.geometry.min_distance_points,
                          config_.geometry.min_distance_points);
        private_nh_.param("forward_sign", config_.geometry.forward_sign,
                          config_.geometry.forward_sign);
        private_nh_.param("left_sign", config_.geometry.left_sign,
                          config_.geometry.left_sign);
        config_.geometry.forward_sign =
            config_.geometry.forward_sign >= 0 ? 1 : -1;
        config_.geometry.left_sign = config_.geometry.left_sign >= 0 ? 1 : -1;
        config_.geometry.distance_percentile = std::max(
            0.0, std::min(1.0, config_.geometry.distance_percentile));
        config_.distance_filter_alpha = std::max(
            0.0, std::min(1.0, config_.distance_filter_alpha));
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        EquipmentGeometryResult result =
            estimateEquipmentGeometry(cloud, config_.geometry);
        applyInputQuality(*msg, cloud.size(), result);
        applyDistanceFilter(result);

        EquipmentState out;
        out.header = msg->header;
        out.header.frame_id = config_.target_frame.empty()
                                  ? msg->header.frame_id
                                  : config_.target_frame;
        out.source = "pointcloud";
        out.attitude_valid = result.attitude_valid;
        out.distances_valid = result.distances_valid;
        out.roll_valid = result.roll_valid;
        out.pitch_valid = result.pitch_valid;
        out.yaw_valid = result.yaw_valid;
        out.left_front_valid = result.left_front_valid;
        out.left_rear_valid = result.left_rear_valid;
        out.right_front_valid = result.right_front_valid;
        out.right_rear_valid = result.right_rear_valid;
        out.roll_deg = result.roll_deg;
        out.pitch_deg = result.pitch_deg;
        out.yaw_deg = result.yaw_deg;
        out.ground_plane_rmse = result.ground_plane_rmse;
        out.left_front_clearance_m = result.left_front_clearance_m;
        out.left_rear_clearance_m = result.left_rear_clearance_m;
        out.right_front_clearance_m = result.right_front_clearance_m;
        out.right_rear_clearance_m = result.right_rear_clearance_m;
        out.left_front_mm = result.left_front_mm;
        out.left_rear_mm = result.left_rear_mm;
        out.right_front_mm = result.right_front_mm;
        out.right_rear_mm = result.right_rear_mm;
        out.ground_points = result.ground_points;
        out.wall_points = result.wall_points;
        out.left_front_points = result.left_front_points;
        out.left_rear_points = result.left_rear_points;
        out.right_front_points = result.right_front_points;
        out.right_rear_points = result.right_rear_points;
        out.point_count = static_cast<std::uint32_t>(cloud.size());
        out.quality = result.quality;
        out.invalid_reason = result.invalid_reason;
        pub_.publish(out);
    }

    void applyInputQuality(const sensor_msgs::PointCloud2& msg,
                           std::size_t point_count,
                           EquipmentGeometryResult& result) const {
        if (point_count == 0U) {
            invalidateAll("NO_POINTCLOUD", result);
            return;
        }
        if (point_count < static_cast<std::size_t>(config_.geometry.min_total_points)) {
            invalidateAll("LOW_TOTAL_POINTS", result);
            return;
        }
        if (!config_.required_frame_id.empty() &&
            msg.header.frame_id != config_.required_frame_id) {
            invalidateAll("FRAME_ID_INVALID", result);
            return;
        }
        if (config_.max_pointcloud_age_sec > 0.0 && !msg.header.stamp.isZero()) {
            const double age = (ros::Time::now() - msg.header.stamp).toSec();
            if (std::isfinite(age) && age > config_.max_pointcloud_age_sec) {
                invalidateAll("POINTCLOUD_STALE", result);
            }
        }
    }

    void invalidateAll(const std::string& reason,
                       EquipmentGeometryResult& result) const {
        result.attitude_valid = false;
        result.distances_valid = false;
        result.roll_valid = false;
        result.pitch_valid = false;
        result.yaw_valid = false;
        result.left_front_valid = false;
        result.left_rear_valid = false;
        result.right_front_valid = false;
        result.right_rear_valid = false;
        result.quality = "LOST";
        result.invalid_reason = reason;
    }

    void applyDistanceFilter(EquipmentGeometryResult& result) {
        filterDistance(result.left_front_valid, result.left_front_mm,
                       last_left_front_mm_);
        filterDistance(result.left_rear_valid, result.left_rear_mm,
                       last_left_rear_mm_);
        filterDistance(result.right_front_valid, result.right_front_mm,
                       last_right_front_mm_);
        filterDistance(result.right_rear_valid, result.right_rear_mm,
                       last_right_rear_mm_);
        result.distances_valid = result.left_front_valid &&
                                 result.left_rear_valid &&
                                 result.right_front_valid &&
                                 result.right_rear_valid;
        result.left_front_clearance_m = result.left_front_mm / 1000.0;
        result.left_rear_clearance_m = result.left_rear_mm / 1000.0;
        result.right_front_clearance_m = result.right_front_mm / 1000.0;
        result.right_rear_clearance_m = result.right_rear_mm / 1000.0;
        if (result.invalid_reason == "none" && !result.distances_valid) {
            result.quality = result.attitude_valid ? "DEGRADED" : "INVALID";
            result.invalid_reason = "DISTANCE_JUMP_REJECTED";
        }
    }

    void filterDistance(bool& valid, double& value_mm, double& last_mm) const {
        if (!valid) {
            return;
        }
        if (std::isfinite(last_mm)) {
            const double jump_m = std::abs(value_mm - last_mm) / 1000.0;
            if (config_.max_distance_jump_m > 0.0 &&
                jump_m > config_.max_distance_jump_m) {
                valid = false;
                return;
            }
            value_mm = config_.distance_filter_alpha * value_mm +
                       (1.0 - config_.distance_filter_alpha) * last_mm;
        }
        last_mm = value_mm;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    NodeConfig config_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double last_left_front_mm_ = std::numeric_limits<double>::quiet_NaN();
    double last_left_rear_mm_ = std::numeric_limits<double>::quiet_NaN();
    double last_right_front_mm_ = std::numeric_limits<double>::quiet_NaN();
    double last_right_rear_mm_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace
}  // namespace target_localizer

int main(int argc, char** argv) {
    ros::init(argc, argv, "equipment_state_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    target_localizer::EquipmentStateNode node(nh, private_nh);
    ros::spin();
    return 0;
}
