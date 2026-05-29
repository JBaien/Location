#include <algorithm>
#include <cstdint>
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
        private_nh_.param("max_ground_plane_rmse",
                          config_.geometry.max_ground_plane_rmse,
                          config_.geometry.max_ground_plane_rmse);
        private_nh_.param("min_ground_normal_z",
                          config_.geometry.min_ground_normal_z,
                          config_.geometry.min_ground_normal_z);
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
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        const EquipmentGeometryResult result =
            estimateEquipmentGeometry(cloud, config_.geometry);

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
        pub_.publish(out);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    NodeConfig config_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
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
