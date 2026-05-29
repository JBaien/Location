#ifndef LIDAR_FUSION_H
#define LIDAR_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace lidar_fusion
{

// 自定义点类型，精确匹配您提供的点云数据结构
// offset对应: x(0), y(4), z(8), intensity(16), ring(20), time(24)
struct EIGEN_ALIGN16 PointXYZIRT
{
    float x;                        // x坐标 - offset: 0
    float y;                        // y坐标 - offset: 4
    float z;                        // z坐标 - offset: 8
    float _padding;                 // 填充字节 - offset: 12
    float intensity;                // 强度 - offset: 16
    std::uint16_t ring;             // 环号 - offset: 20
    std::uint16_t _padding2;        // 填充字节 - offset: 22
    float time;                     // 时间戳 - offset: 24
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

// 注册点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_fusion::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

namespace lidar_fusion
{

class LidarFusion
{
public:
    LidarFusion(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~LidarFusion();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 参数
    std::string lslidar_topic_;
    std::string velodyne_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    double tf_tolerance_;
    
    // 发布器
    ros::Publisher fused_cloud_pub_;
    
    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 消息过滤器和同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lslidar_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> velodyne_sub_;
    std::shared_ptr<Synchronizer> sync_;
    
    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& lslidar_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& velodyne_msg);
    
    // 工具函数
    void loadParameters();
    bool transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                           const std::string& target_frame,
                           sensor_msgs::PointCloud2& transformed_cloud);
    void fusePointClouds(const sensor_msgs::PointCloud2::ConstPtr& cloud1,
                        const sensor_msgs::PointCloud2::ConstPtr& cloud2,
                        sensor_msgs::PointCloud2& fused_cloud);
};

}

#endif // LIDAR_FUSION_H