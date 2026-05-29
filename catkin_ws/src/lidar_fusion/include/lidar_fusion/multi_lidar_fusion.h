#ifndef MULTI_LIDAR_FUSION_H
#define MULTI_LIDAR_FUSION_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lidar_fusion/lidar_fusion.h"

namespace lidar_fusion {

class MultiLidarFusion {
public:
    MultiLidarFusion(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~MultiLidarFusion() = default;

private:
    using CloudMsg = sensor_msgs::PointCloud2;
    using CloudConstPtr = sensor_msgs::PointCloud2::ConstPtr;
    using PointCloud = pcl::PointCloud<PointXYZIRT>;
    using SyncPolicy2 =
        message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
    using SyncPolicy3 =
        message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg,
                                                        CloudMsg>;
    using Synchronizer2 = message_filters::Synchronizer<SyncPolicy2>;
    using Synchronizer3 = message_filters::Synchronizer<SyncPolicy3>;

    struct Config {
        int lidar_num = 3;
        std::vector<std::string> lidar_topics;
        std::string output_topic = "/points_raw";
        std::string output_frame_id = "base_link";
        int sync_queue_size = 20;
        double sync_slop = 0.05;
        double tf_timeout = 0.05;
        bool drop_empty_cloud = true;
        int min_points_per_lidar = 10;
        bool normalize_point_time = true;
        bool enable_voxel_filter = false;
        double voxel_leaf_size = 0.05;
        bool enable_diagnostics = true;
        double diagnostics_period = 1.0;
    };

    struct Diagnostics {
        uint64_t sync_callbacks = 0;
        uint64_t published = 0;
        uint64_t dropped_empty = 0;
        uint64_t dropped_min_points = 0;
        uint64_t dropped_tf = 0;
        uint64_t dropped_conversion = 0;
        uint64_t dropped_field = 0;
        uint64_t dropped_exception = 0;
        uint64_t voxel_filtered = 0;
        ros::Time last_publish_stamp;
        ros::Time last_callback_wall_time;
        std::vector<size_t> last_input_points;
        std::vector<std::string> last_input_frames;
        size_t last_output_points = 0;
        double last_sync_span = 0.0;
    };

    void loadParameters();
    bool readStringVectorParam(const std::string& key,
                               std::vector<std::string>& value) const;
    template <typename T>
    void readParam(const std::string& key, T& value) const;
    void validateConfig();
    void initializeSubscribers();
    void diagnosticsTimerCallback(const ros::TimerEvent&);

    void callback2(const CloudConstPtr& cloud1, const CloudConstPtr& cloud2);
    void callback3(const CloudConstPtr& cloud1, const CloudConstPtr& cloud2,
                   const CloudConstPtr& cloud3);
    void processClouds(const std::vector<CloudConstPtr>& clouds);

    bool validateInputCloud(const CloudConstPtr& cloud, size_t index,
                            size_t& point_count);
    bool hasRequiredFields(const CloudMsg& cloud) const;
    bool transformCloudToOutputFrame(const CloudConstPtr& cloud,
                                     const ros::Time& output_stamp,
                                     PointCloud& transformed_cloud);
    bool fuseTransformedClouds(const std::vector<PointCloud::Ptr>& clouds,
                               const ros::Time& output_stamp,
                               CloudMsg& output_msg);
    void applyVoxelFilter(PointCloud::Ptr& cloud);
    ros::Time chooseOutputStamp(const std::vector<CloudConstPtr>& clouds) const;
    double computeSyncSpan(const std::vector<CloudConstPtr>& clouds) const;
    void updateInputDiagnostics(const std::vector<CloudConstPtr>& clouds,
                                const std::vector<size_t>& point_counts,
                                double sync_span);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    Config config_;
    Diagnostics diagnostics_;

    ros::Publisher fused_cloud_pub_;
    ros::Timer diagnostics_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<std::shared_ptr<message_filters::Subscriber<CloudMsg>>>
        subscribers_;
    std::shared_ptr<Synchronizer2> sync2_;
    std::shared_ptr<Synchronizer3> sync3_;
};

}  // namespace lidar_fusion

#endif  // MULTI_LIDAR_FUSION_H
