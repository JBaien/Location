#include <ros/ros.h>
#include "lidar_fusion/lidar_fusion.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_fusion_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    try
    {
        // 创建点云融合对象
        lidar_fusion::LidarFusion fusion(nh, private_nh);
        
        ROS_INFO("Lidar fusion node started successfully");
        
        // 进入消息循环
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error in lidar_fusion_node: %s", e.what());
        return -1;
    }
    
    ROS_INFO("Lidar fusion node shutting down");
    return 0;
}