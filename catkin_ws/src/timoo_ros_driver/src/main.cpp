
// #include "pcl/impl/point_types.hpp"
#include <cmath>
#include <cstdint>
#include <memory>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
  
#include <pcl/io/pcd_io.h> 
#include <string>
#include <thread>

#include "common/common.h"
#include "common/device_config.h"
#include "common/point_type.h"
#include "app/timoo_driver.h"
#include "ros/init.h"
#include <sensor_msgs/Imu.h>

using namespace timoo::driver;
bool remove_invalid_points;

ros::Time ResolveCloudStamp(const base::TimooPointCloudPtr& points_data,
                            bool use_sensor_timestamp) {
    if (use_sensor_timestamp && points_data &&
        std::isfinite(points_data->timestamp) && points_data->timestamp > 0.0) {
        return ros::Time(points_data->timestamp);
    }
    return ros::Time::now();
}


struct TimooPointXYZIRT {
 PCL_ADD_POINT4D;
 float intensity;
 std::uint16_t ring = 0;
 float time = 0;
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 } EIGEN_ALIGN16;
 

 POINT_CLOUD_REGISTER_POINT_STRUCT(
 TimooPointXYZIRT, 
 (float, x, x)
 (float, y, y)
 (float, z, z)
 (float, intensity, intensity)
 (std::uint16_t, ring, ring)
 (float, time, time)
 )
 typedef pcl::PointCloud<TimooPointXYZIRT> TimooPointCloudXYZIRT;

void AddPointCloud(const base::TimooPointCloudPtr& points_data,
                   pcl::PointCloud<TimooPointXYZIRT>& cloud,
                   const ros::Publisher& points_pub_,
                   const std::string& frame_id,
                   bool use_sensor_timestamp) {
  for (const auto& p : points_data->points) {
    if ((!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) && remove_invalid_points) {
      continue;
    }

    TimooPointXYZIRT point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring_id;
    point.time = p.time;
    cloud.push_back(point);
  }

    if(cloud.size() < 5){return;}
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = ResolveCloudStamp(points_data, use_sensor_timestamp);
    points_pub_.publish(output);
    //std::cout << "time stamp == " << ros::Time::now()<< std::endl;
}

void AddPointCloud(const base::TimooPointCloudPtr& points_data,
                   pcl::PointCloud<pcl::PointXYZI>& cloud,
                   const ros::Publisher& points_pub_,
                   const std::string& frame_id,
                   bool use_sensor_timestamp) {
  for (const auto& p : points_data->points) {
    if ((!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) && remove_invalid_points) {
      continue;
    }

    pcl::PointXYZI point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    cloud.push_back(point);
  }

    if(cloud.size() < 5){return;}
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = ResolveCloudStamp(points_data, use_sensor_timestamp);
    points_pub_.publish(output);
    //std::cout << "time stamp == " << ros::Time::now()<< std::endl;
}

int main(int argc, char **argv) {

    // ROS init
    ros::init(argc, argv, "timoo_ros_driver_node");
    ros::NodeHandle nh;

    double cut_angle, min_distance, max_distance;
    int udp_port, status_port,imu_port;
    bool fixed_points_count, use_imu_data, use_gps_clock, use_tail_time;
    bool use_sensor_timestamp;
    std::string lidar_type, host_ip, frame_id;

    nh.param("cut_angle", cut_angle, 0.0);
    nh.param("udp_port", udp_port, 2368);
    nh.param("status_port", status_port, 8603);
    nh.param("fixed_points_count", fixed_points_count, false);
    nh.param("remove_invalid_points", remove_invalid_points, false);
    nh.param("lidar_type", lidar_type, std::string("TIMOO32"));
    nh.param("host_ip", host_ip, std::string("192.168.1.106"));
    nh.param("frame_id", frame_id, std::string("timoo"));
    nh.param("min_distance", min_distance, 0.4);
    nh.param("max_distance", max_distance, 150.0);
    nh.param("imu_port", imu_port, 7788);
    nh.param("use_imu_data", use_imu_data, false);
    nh.param("use_gps_clock", use_gps_clock, false);
    nh.param("use_tail_time", use_tail_time, false);
    nh.param("use_sensor_timestamp", use_sensor_timestamp, false);

    

    ros::Publisher points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("timoo_points", 1);
    ros::Publisher imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    // 外部回调函数
    auto PubFunction = [points_pub_, frame_id, use_sensor_timestamp](base::TimooPointCloudPtr points_data) {
        std::cout << "pub points num:" << points_data->points.size() << std::endl;
        // if (points_data->points.size() > 100000) {
            // std::cout << "------------------------------------------" << std::endl;
        // }
        // std::cout << "frame timestamp:" << std::to_string(points_data->timestamp) << std::endl;
        // std::cout << "last timestamp:" << std::to_string(points_data->points.back().time) << std::endl;
        // std::cout << "first timestamp:" << std::to_string(points_data->points.front().time) << std::endl;
        #if defined(POINT_TYPE_XYZI)
            pcl::PointCloud<pcl::PointXYZI> cloud;
            AddPointCloud(points_data, cloud, points_pub_, frame_id, use_sensor_timestamp);
            //std::cout << "POINTXYZIRT cloud size() === " << cloud.size() <<std::endl; 
        #endif

        #if defined(POINT_TYPE_XYZIRT)
            pcl::PointCloud<TimooPointXYZIRT> cloud;
            AddPointCloud(points_data, cloud, points_pub_, frame_id, use_sensor_timestamp);
            //std::cout << "POINTXYZIRT cloud size() === " << cloud.size() <<std::endl; 
        #endif
    };

    auto PubFunction_imu = [imu_pub_, frame_id](base::TimooIMUPtr imu_data) {
        // 从传感器获取数据 - 这里用伪代码表示
        // 实际应用中需要替换为您的实际数据读取代码
        float Accel_x = imu_data->Accel_x;
        float Accel_y = imu_data->Accel_y;
        float Accel_z = imu_data->Accel_z;
        float Gyro_x = imu_data->Gyro_x;
        float Gyro_y = imu_data->Gyro_y;
        float Gyro_z = imu_data->Gyro_z;
        float temperature = imu_data->temperature;

        // 创建IMU消息
        sensor_msgs::Imu imu_msg;
        
        // 设置消息头和时间戳
        imu_msg.header.stamp = ros::Time::now(); // 使用ROS当前时间
        imu_msg.header.frame_id = frame_id; // 坐标系名称
        
        // 温度填入x
        imu_msg.orientation.x = temperature;
        
        // 转换加速度数据（原始数据为16位有符号整数）
        imu_msg.linear_acceleration.x = Accel_x;
        imu_msg.linear_acceleration.y = Accel_y;
        imu_msg.linear_acceleration.z = Accel_z;
        
        // 转换陀螺仪数据
        imu_msg.angular_velocity.x = Gyro_x;
        imu_msg.angular_velocity.y = Gyro_y;
        imu_msg.angular_velocity.z = Gyro_z;
        
        // 设置协方差矩阵（根据传感器精度调整）
        // 加速度协方差（假设各轴独立，方差为0.01 m²/s⁴）
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        
        // 陀螺仪协方差（假设各轴独立，方差为0.0001 rad²/s²）
        imu_msg.angular_velocity_covariance[0] = 0.0001;
        imu_msg.angular_velocity_covariance[4] = 0.0001;
        imu_msg.angular_velocity_covariance[8] = 0.0001;
        
        // 姿态四元数（未提供，设置为0并标记为无效）
        imu_msg.orientation_covariance[0] = -1; // 表示无效姿态
                // 发布消息
        imu_pub_.publish(imu_msg);
          //std::cout << "shoudao imu " << std::endl;
    };

    base::DeviceConfig device_config;
    device_config.cut_angle =cut_angle;
    device_config.udp_port = udp_port;
    device_config.status_port = status_port;
    device_config.fixed_points_count = fixed_points_count;
    device_config.use_imu_data = use_imu_data;
    device_config.use_gps_clock = use_gps_clock;
    device_config.use_tail_time = use_tail_time;
    // device_config.lidar_type = device_config.lidar_type;
        // 如果 lidar_type 是 enum，可以做字符串转换：
    if (lidar_type == "TIMOO16") {
        device_config.lidar_type = base::SensorType::TIMOO16;
    } else if (lidar_type == "TIMOO32") {
        device_config.lidar_type = base::SensorType::TIMOO32;
    } else if (lidar_type == "TIMOO1550"){
        device_config.lidar_type = base::SensorType::TIMOO1550;
    } else if (lidar_type == "TIMOO1550STD"){
        device_config.lidar_type = base::SensorType::TIMOO1550STD;
    } else if (lidar_type == "TIMOO128"){
        device_config.lidar_type = base::SensorType::TIMOO128;
    }  else if (lidar_type == "TIMOO128V"){
        device_config.lidar_type = base::SensorType::TIMOO128V;
    } 
    else {
        ROS_WARN("Unknown lidar_type: %s", lidar_type.c_str());
    }

    device_config.min_distance = min_distance;
    device_config.max_distance = max_distance;

    device_config.imu_dst_port = imu_port;
    device_config.host_ip = host_ip;
    std::cout << "********************************雷达参数**********************************************" << std::endl; 
    std::cout << "雷达型号：" << lidar_type << std::endl;
    std::cout << "本机ip" << host_ip << std::endl;
    std::cout << "分帧角度：" << device_config.cut_angle << std::endl;
    std::cout << "雷达数据包端口：" << device_config.udp_port << std::endl;
    std::cout << "雷达设备信息包端口：" << device_config.status_port << std::endl;
    std::cout << "imu端口 : " << device_config.imu_dst_port << std::endl;
    std::cout << "点云矩阵组织：" << std::to_string(device_config.fixed_points_count) << std::endl;
    std::cout << "雷达最小探测距离：" << device_config.min_distance << std::endl;
    std::cout << "雷达最大探测距离：" << device_config.max_distance << std::endl;
    std::cout << "******************************************************************************************" << std::endl;

    TimooDriver timoo_driver;
    timoo_driver.Init(device_config);
    timoo_driver.registerFrameCloudCallBack(PubFunction);
    timoo_driver.registerFrameImuCallBack(PubFunction_imu);
    timoo_driver.Start();


    ros::spin();

}
