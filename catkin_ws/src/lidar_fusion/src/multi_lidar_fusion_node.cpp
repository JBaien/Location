#include <ros/ros.h>

#include "lidar_fusion/multi_lidar_fusion.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_lidar_fusion_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try {
        lidar_fusion::MultiLidarFusion fusion(nh, private_nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("multi_lidar_fusion_node failed to start: "
                         << e.what());
        return 1;
    }

    return 0;
}
