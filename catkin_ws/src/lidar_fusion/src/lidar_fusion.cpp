#include "lidar_fusion/lidar_fusion.h"
#include <boost/make_shared.hpp>

namespace lidar_fusion
{

LidarFusion::LidarFusion(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_)
{
    try
    {
        // 加载参数
        loadParameters();
        
        // 初始化发布器
        fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        
        // 初始化订阅器和同步器
        lslidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, lslidar_topic_, 10));
        velodyne_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, velodyne_topic_, 10));
        
        sync_.reset(new Synchronizer(SyncPolicy(10), *lslidar_sub_, *velodyne_sub_));
        sync_->registerCallback(boost::bind(&LidarFusion::cloudCallback, this, _1, _2));
        
        ROS_INFO("LidarFusion initialized");
        ROS_INFO("  LSLidar topic: %s", lslidar_topic_.c_str());
        ROS_INFO("  Velodyne topic: %s", velodyne_topic_.c_str());
        ROS_INFO("  Output topic: %s", output_topic_.c_str());
        ROS_INFO("  Output frame_id: %s", output_frame_id_.c_str());
        ROS_INFO("  TF tolerance: %.3f", tf_tolerance_);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("LidarFusion初始化失败: %s", e.what());
        throw;
    }
}

LidarFusion::~LidarFusion()
{
}

void LidarFusion::loadParameters()
{
    // 从参数服务器获取参数，如果没有设置则使用默认值
    private_nh_.param<std::string>("lslidar_topic", lslidar_topic_, "/cx/lslidar_point_cloud");
    private_nh_.param<std::string>("velodyne_topic", velodyne_topic_, "/velodyne_points");
    private_nh_.param<std::string>("output_topic", output_topic_, "/fused_point_cloud");
    private_nh_.param<std::string>("output_frame_id", output_frame_id_, "velodyne");
    private_nh_.param<double>("tf_tolerance", tf_tolerance_, 0.1);
    
    // 参数验证
    if (lslidar_topic_.empty() || velodyne_topic_.empty() || output_topic_.empty())
    {
        ROS_ERROR("Topic名称不能为空");
        throw std::runtime_error("Invalid topic names");
    }
    
    if (output_frame_id_.empty())
    {
        ROS_ERROR("Output frame_id不能为空");
        throw std::runtime_error("Invalid output frame_id");
    }
    
    if (tf_tolerance_ <= 0.0 || tf_tolerance_ > 10.0)
    {
        ROS_WARN("TF容忍度参数不合理(%.3f)，使用默认值0.1", tf_tolerance_);
        tf_tolerance_ = 0.1;
    }
}

void LidarFusion::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& lslidar_msg,
                               const sensor_msgs::PointCloud2::ConstPtr& velodyne_msg)
{
    // 数据验证
    if (!lslidar_msg || !velodyne_msg)
    {
        ROS_ERROR("收到空的点云数据指针");
        return;
    }
    
    // 时间戳验证
    if (lslidar_msg->header.stamp.isZero() || velodyne_msg->header.stamp.isZero())
    {
        ROS_WARN("点云数据时间戳为零");
        return;
    }
    
    // 检查时间戳差异是否过大
    double time_diff = std::abs((lslidar_msg->header.stamp - velodyne_msg->header.stamp).toSec());
    if (time_diff > tf_tolerance_)
    {
        ROS_WARN("Point cloud time difference: %.3f秒", time_diff);
    }
    
    if (lslidar_msg->width * lslidar_msg->height == 0 || velodyne_msg->width * velodyne_msg->height == 0)
    {
        ROS_WARN("收到空的点云数据");
        return;
    }
    
    ROS_DEBUG("收到同步的两个点云数据: /cx/lslidar_point_cloud(%u点) + /velodyne_points(%u点)", 
             (unsigned int)(lslidar_msg->width * lslidar_msg->height),
             (unsigned int)(velodyne_msg->width * velodyne_msg->height));
    
    try
    {
        // 验证frame_id
        if (lslidar_msg->header.frame_id.empty() || velodyne_msg->header.frame_id.empty())
        {
            ROS_ERROR("点云数据frame_id为空");
            return;
        }
        
        // 步骤1: 处理lslidar点云坐标系
        sensor_msgs::PointCloud2 transformed_lslidar;
        if (!transformPointCloud(lslidar_msg, output_frame_id_, transformed_lslidar))
        {
            ROS_WARN("Failed to transform lslidar point cloud to frame %s", output_frame_id_.c_str());
            return;
        }
        
        // 步骤2: 处理velodyne点云坐标系
        sensor_msgs::PointCloud2 processed_velodyne;
        if (!transformPointCloud(velodyne_msg, output_frame_id_, processed_velodyne))
        {
            ROS_WARN("Failed to transform velodyne point cloud to frame %s", output_frame_id_.c_str());
            return;
        }
        
        // 步骤3: 融合两个处理后的点云
        sensor_msgs::PointCloud2 fused_cloud;
        // 创建ConstPtr来调用fusePointClouds
        sensor_msgs::PointCloud2::ConstPtr transformed_lslidar_ptr = boost::make_shared<sensor_msgs::PointCloud2>(transformed_lslidar);
        sensor_msgs::PointCloud2::ConstPtr processed_velodyne_ptr = boost::make_shared<sensor_msgs::PointCloud2>(processed_velodyne);
        fusePointClouds(transformed_lslidar_ptr, processed_velodyne_ptr, fused_cloud);
        
        // 验证融合结果
        if (fused_cloud.width * fused_cloud.height == 0)
        {
            ROS_ERROR("点云融合失败，结果为空");
            return;
        }
        
        // 步骤4: 发布融合后的点云到 /fused_point_cloud topic
        fused_cloud_pub_.publish(fused_cloud);
        
        ROS_DEBUG("成功融合并发布点云数据，总计 %d 个点", 
                 (int)(fused_cloud.width * fused_cloud.height));
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error in cloudCallback: %s", e.what());
    }
}

bool LidarFusion::transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                     const std::string& target_frame,
                                     sensor_msgs::PointCloud2& transformed_cloud)
{
    try
    {
        // 输入验证
        if (!cloud_msg)
        {
            ROS_ERROR("输入点云消息为空指针");
            return false;
        }
        
        if (cloud_msg->header.frame_id.empty())
        {
            ROS_ERROR("输入点云frame_id为空");
            return false;
        }
        
        if (target_frame.empty())
        {
            ROS_ERROR("目标frame_id为空");
            return false;
        }
        
        // 如果源坐标系与目标坐标系相同，直接复制
        if (cloud_msg->header.frame_id == target_frame)
        {
            ROS_DEBUG("坐标系相同(%s)，无需变换", target_frame.c_str());
            transformed_cloud = *cloud_msg;
            return true;
        }
        
        // 检查变换是否可用
        ros::Time transform_time = cloud_msg->header.stamp;
        if (transform_time.isZero())
        {
            transform_time = ros::Time::now();
            ROS_WARN("使用当前时间进行TF查找");
        }
        
        if (!tf_buffer_.canTransform(target_frame, cloud_msg->header.frame_id, 
                                    transform_time, ros::Duration(tf_tolerance_)))
        {
            ROS_WARN("Transform from %s to %s not available at time %.3f", 
                    cloud_msg->header.frame_id.c_str(), target_frame.c_str(), 
                    transform_time.toSec());
            return false;
        }
        
        // 获取变换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            target_frame, cloud_msg->header.frame_id, transform_time, 
            ros::Duration(tf_tolerance_));
        
        // 转换为PCL点云
        pcl::PointCloud<PointXYZIRT>::Ptr pcl_cloud(new pcl::PointCloud<PointXYZIRT>);
        try
        {
            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("PCL转换失败: %s", e.what());
            return false;
        }
        
        // 验证点云不为空
        if (pcl_cloud->empty())
        {
            ROS_WARN("PCL点云为空");
            return false;
        }
        
        // 创建变换矩阵
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);

        // 应用变换 - 使用逐点变换以兼容自定义点类型
        pcl::PointCloud<PointXYZIRT>::Ptr transformed_pcl_cloud(new pcl::PointCloud<PointXYZIRT>);
        transformed_pcl_cloud->points.resize(pcl_cloud->points.size());
        transformed_pcl_cloud->header = pcl_cloud->header;

        for (size_t i = 0; i < pcl_cloud->points.size(); ++i)
        {
            Eigen::Vector3f point(pcl_cloud->points[i].x,
                                 pcl_cloud->points[i].y,
                                 pcl_cloud->points[i].z);
            Eigen::Vector3f transformed_point = transform_eigen.cast<float>() * point;

            transformed_pcl_cloud->points[i].x = transformed_point.x();
            transformed_pcl_cloud->points[i].y = transformed_point.y();
            transformed_pcl_cloud->points[i].z = transformed_point.z();
            transformed_pcl_cloud->points[i].intensity = pcl_cloud->points[i].intensity;
            transformed_pcl_cloud->points[i].ring = pcl_cloud->points[i].ring;
            transformed_pcl_cloud->points[i].time = pcl_cloud->points[i].time;
        }
        
        // 转换回ROS消息
        pcl::toROSMsg(*transformed_pcl_cloud, transformed_cloud);
        transformed_cloud.header.frame_id = target_frame;
        transformed_cloud.header.stamp = cloud_msg->header.stamp;
        
        ROS_DEBUG("成功变换点云: %s -> %s (%u点)", 
                 cloud_msg->header.frame_id.c_str(), target_frame.c_str(), 
                 (unsigned int)transformed_pcl_cloud->size());
        
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform failed: %s", ex.what());
        return false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error in transformPointCloud: %s", e.what());
        return false;
    }
}

void LidarFusion::fusePointClouds(const sensor_msgs::PointCloud2::ConstPtr& cloud1,
                                 const sensor_msgs::PointCloud2::ConstPtr& cloud2,
                                 sensor_msgs::PointCloud2& fused_cloud)
{
    try
    {
        // 输入验证
        if (!cloud1 || !cloud2)
        {
            ROS_ERROR("输入点云数据为空指针");
            return;
        }
        
        size_t cloud1_points = cloud1->width * cloud1->height;
        size_t cloud2_points = cloud2->width * cloud2->height;
        
        if (cloud1_points == 0 || cloud2_points == 0)
        {
            ROS_WARN("输入点云数据为空");
            return;
        }
        
        ROS_DEBUG("开始融合两个点云: cloud1(%u点) + cloud2(%u点)", 
                (unsigned int)cloud1_points, (unsigned int)cloud2_points);
        
        // 步骤1: 转换为PCL点云格式
        pcl::PointCloud<PointXYZIRT>::Ptr pcl_cloud1(new pcl::PointCloud<PointXYZIRT>);
        pcl::PointCloud<PointXYZIRT>::Ptr pcl_cloud2(new pcl::PointCloud<PointXYZIRT>);
        
        try
        {
            pcl::fromROSMsg(*cloud1, *pcl_cloud1);  // cloud1是变换后的lslidar点云
            pcl::fromROSMsg(*cloud2, *pcl_cloud2);  // cloud2是处理后的velodyne点云
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("PCL转换失败: %s", e.what());
            return;
        }
        
        // 验证PCL点云不为空
        if (pcl_cloud1->empty() || pcl_cloud2->empty())
        {
            ROS_WARN("PCL点云为空");
            return;
        }
        
        ROS_DEBUG("PCL转换完成: 变换后lslidar(%u点) + 处理后velodyne(%u点)", 
                (unsigned int)pcl_cloud1->size(), (unsigned int)pcl_cloud2->size());
        
        // 步骤2: 创建融合后的点云容器
        pcl::PointCloud<PointXYZIRT>::Ptr fused_pcl_cloud(new pcl::PointCloud<PointXYZIRT>);
        
        // 步骤3: 进行点云融合 - 关键融合逻辑
        try
        {
            *fused_pcl_cloud = *pcl_cloud1;          // 先复制变换后的lslidar点云
            *fused_pcl_cloud += *pcl_cloud2;         // 再添加处理后的velodyne点云
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("点云融合失败: %s", e.what());
            return;
        }
        
        ROS_DEBUG("点云融合完成: %u + %u = %u 个点", 
                (unsigned int)pcl_cloud1->size(), (unsigned int)pcl_cloud2->size(), (unsigned int)fused_pcl_cloud->size());
        
        // 步骤4: 转换回ROS消息格式
        try
        {
            pcl::toROSMsg(*fused_pcl_cloud, fused_cloud);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("PCL转换回ROS消息失败: %s", e.what());
            return;
        }
        
        fused_cloud.header.frame_id = output_frame_id_;
        
        // 使用较新的时间戳
        if (cloud1->header.stamp > cloud2->header.stamp)
            fused_cloud.header.stamp = cloud1->header.stamp;
        else
            fused_cloud.header.stamp = cloud2->header.stamp;
            
        ROS_DEBUG("融合结果: frame_id=%s, 点数=%u, 时间戳=%f", 
                 fused_cloud.header.frame_id.c_str(), 
                 (unsigned int)fused_pcl_cloud->size(),
                 fused_cloud.header.stamp.toSec());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error in fusePointClouds: %s", e.what());
    }
}

}  // namespace lidar_fusion
