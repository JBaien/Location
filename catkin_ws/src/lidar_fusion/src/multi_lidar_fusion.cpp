#include "lidar_fusion/multi_lidar_fusion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>

namespace lidar_fusion {

MultiLidarFusion::MultiLidarFusion(ros::NodeHandle& nh,
                                   ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_) {
    loadParameters();
    validateConfig();

    fused_cloud_pub_ =
        nh_.advertise<CloudMsg>(config_.output_topic, 10, false);
    initializeSubscribers();

    diagnostics_.last_input_points.assign(config_.lidar_num, 0);
    diagnostics_.last_input_frames.assign(config_.lidar_num, "");

    if (config_.enable_diagnostics) {
        diagnostics_timer_ =
            private_nh_.createTimer(ros::Duration(config_.diagnostics_period),
                                    &MultiLidarFusion::diagnosticsTimerCallback,
                                    this);
    }

    ROS_INFO_STREAM("MultiLidarFusion initialized: lidar_num="
                    << config_.lidar_num
                    << ", output_topic=" << config_.output_topic
                    << ", output_frame_id=" << config_.output_frame_id
                    << ", sync_slop=" << config_.sync_slop
                    << ", tf_timeout=" << config_.tf_timeout);
    for (size_t i = 0; i < config_.lidar_topics.size(); ++i) {
        ROS_INFO_STREAM("  lidar[" << i << "] topic="
                                   << config_.lidar_topics[i]);
    }
}

template <typename T>
void MultiLidarFusion::readParam(const std::string& key, T& value) const {
    if (private_nh_.getParam(key, value)) {
        return;
    }
    if (nh_.getParam("multi_lidar_fusion/" + key, value)) {
        return;
    }
    ros::param::get("/multi_lidar_fusion/" + key, value);
}

bool MultiLidarFusion::readStringVectorParam(
    const std::string& key, std::vector<std::string>& value) const {
    XmlRpc::XmlRpcValue raw;
    if (!private_nh_.getParam(key, raw) &&
        !nh_.getParam("multi_lidar_fusion/" + key, raw) &&
        !ros::param::get("/multi_lidar_fusion/" + key, raw)) {
        return false;
    }

    if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("Parameter " << key << " must be a string array");
        return false;
    }

    value.clear();
    for (int i = 0; i < raw.size(); ++i) {
        if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR_STREAM("Parameter " << key << "[" << i
                                          << "] must be a string");
            return false;
        }
        value.push_back(static_cast<std::string>(raw[i]));
    }
    return true;
}

void MultiLidarFusion::loadParameters() {
    readParam("lidar_num", config_.lidar_num);
    readStringVectorParam("lidar_topics", config_.lidar_topics);
    readParam("output_topic", config_.output_topic);
    readParam("output_frame_id", config_.output_frame_id);
    readParam("sync_queue_size", config_.sync_queue_size);
    readParam("sync_slop", config_.sync_slop);
    readParam("tf_timeout", config_.tf_timeout);
    readParam("drop_empty_cloud", config_.drop_empty_cloud);
    readParam("min_points_per_lidar", config_.min_points_per_lidar);
    readParam("normalize_point_time", config_.normalize_point_time);
    readParam("enable_voxel_filter", config_.enable_voxel_filter);
    readParam("voxel_leaf_size", config_.voxel_leaf_size);
    readParam("enable_diagnostics", config_.enable_diagnostics);
    readParam("diagnostics_period", config_.diagnostics_period);
}

void MultiLidarFusion::validateConfig() {
    if (config_.lidar_num != 2 && config_.lidar_num != 3) {
        throw std::runtime_error("lidar_num must be 2 or 3");
    }
    if (config_.lidar_topics.size() !=
        static_cast<size_t>(config_.lidar_num)) {
        std::ostringstream oss;
        oss << "lidar_topics size (" << config_.lidar_topics.size()
            << ") must equal lidar_num (" << config_.lidar_num << ")";
        throw std::runtime_error(oss.str());
    }
    for (const auto& topic : config_.lidar_topics) {
        if (topic.empty()) {
            throw std::runtime_error("lidar_topics must not contain empty topic");
        }
    }
    if (config_.output_topic.empty()) {
        throw std::runtime_error("output_topic must not be empty");
    }
    if (config_.output_frame_id.empty()) {
        throw std::runtime_error("output_frame_id must not be empty");
    }
    if (config_.sync_queue_size < 1) {
        ROS_WARN("sync_queue_size < 1, reset to 20");
        config_.sync_queue_size = 20;
    }
    if (config_.sync_slop <= 0.0) {
        ROS_WARN("sync_slop <= 0, reset to 0.05");
        config_.sync_slop = 0.05;
    }
    if (config_.tf_timeout <= 0.0) {
        ROS_WARN("tf_timeout <= 0, reset to 0.05");
        config_.tf_timeout = 0.05;
    }
    if (config_.min_points_per_lidar < 0) {
        config_.min_points_per_lidar = 0;
    }
    if (config_.voxel_leaf_size <= 0.0) {
        ROS_WARN("voxel_leaf_size <= 0, reset to 0.05");
        config_.voxel_leaf_size = 0.05;
    }
    if (config_.diagnostics_period <= 0.0) {
        config_.diagnostics_period = 1.0;
    }
}

void MultiLidarFusion::initializeSubscribers() {
    subscribers_.reserve(config_.lidar_num);
    for (const auto& topic : config_.lidar_topics) {
        subscribers_.push_back(
            std::make_shared<message_filters::Subscriber<CloudMsg>>(
                nh_, topic, config_.sync_queue_size,
                ros::TransportHints().tcpNoDelay(true)));
    }

    if (config_.lidar_num == 2) {
        sync2_ = std::make_shared<Synchronizer2>(
            SyncPolicy2(config_.sync_queue_size), *subscribers_[0],
            *subscribers_[1]);
        sync2_->setMaxIntervalDuration(ros::Duration(config_.sync_slop));
        sync2_->registerCallback(
            boost::bind(&MultiLidarFusion::callback2, this, _1, _2));
    } else {
        sync3_ = std::make_shared<Synchronizer3>(
            SyncPolicy3(config_.sync_queue_size), *subscribers_[0],
            *subscribers_[1], *subscribers_[2]);
        sync3_->setMaxIntervalDuration(ros::Duration(config_.sync_slop));
        sync3_->registerCallback(
            boost::bind(&MultiLidarFusion::callback3, this, _1, _2, _3));
    }
}

void MultiLidarFusion::callback2(const CloudConstPtr& cloud1,
                                 const CloudConstPtr& cloud2) {
    processClouds({cloud1, cloud2});
}

void MultiLidarFusion::callback3(const CloudConstPtr& cloud1,
                                 const CloudConstPtr& cloud2,
                                 const CloudConstPtr& cloud3) {
    processClouds({cloud1, cloud2, cloud3});
}

void MultiLidarFusion::processClouds(
    const std::vector<CloudConstPtr>& clouds) {
    ++diagnostics_.sync_callbacks;
    diagnostics_.last_callback_wall_time = ros::Time::now();

    try {
        std::vector<size_t> point_counts(clouds.size(), 0);
        for (size_t i = 0; i < clouds.size(); ++i) {
            if (!validateInputCloud(clouds[i], i, point_counts[i])) {
                return;
            }
        }

        const ros::Time output_stamp = chooseOutputStamp(clouds);
        const double sync_span = computeSyncSpan(clouds);
        updateInputDiagnostics(clouds, point_counts, sync_span);

        std::vector<PointCloud::Ptr> transformed_clouds;
        transformed_clouds.reserve(clouds.size());
        for (const auto& cloud : clouds) {
            PointCloud::Ptr transformed(new PointCloud);
            if (!transformCloudToOutputFrame(cloud, output_stamp,
                                             *transformed)) {
                ++diagnostics_.dropped_tf;
                return;
            }
            transformed_clouds.push_back(transformed);
        }

        CloudMsg output_msg;
        if (!fuseTransformedClouds(transformed_clouds, output_stamp,
                                   output_msg)) {
            return;
        }

        fused_cloud_pub_.publish(output_msg);
        diagnostics_.last_publish_stamp = output_msg.header.stamp;
        diagnostics_.last_output_points =
            static_cast<size_t>(output_msg.width) * output_msg.height;
        ++diagnostics_.published;
    } catch (const std::exception& e) {
        ++diagnostics_.dropped_exception;
        ROS_ERROR_STREAM("Multi lidar fusion callback failed: " << e.what());
    }
}

bool MultiLidarFusion::validateInputCloud(const CloudConstPtr& cloud,
                                          size_t index,
                                          size_t& point_count) {
    if (!cloud) {
        ++diagnostics_.dropped_empty;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] cloud pointer is null");
        return false;
    }
    point_count = static_cast<size_t>(cloud->width) * cloud->height;
    if (cloud->header.stamp.isZero()) {
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] stamp is zero");
        ++diagnostics_.dropped_empty;
        return false;
    }
    if (cloud->header.frame_id.empty()) {
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] frame_id is empty");
        ++diagnostics_.dropped_tf;
        return false;
    }
    if (point_count == 0 && config_.drop_empty_cloud) {
        ++diagnostics_.dropped_empty;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] cloud is empty");
        return false;
    }
    if (point_count < static_cast<size_t>(config_.min_points_per_lidar)) {
        ++diagnostics_.dropped_min_points;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index << "] has only "
                                                    << point_count
                                                    << " points");
        return false;
    }
    if (!hasRequiredFields(*cloud)) {
        ++diagnostics_.dropped_field;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] does not contain "
                                                       "x/y/z/intensity/ring/time");
        return false;
    }
    return true;
}

bool MultiLidarFusion::hasRequiredFields(const CloudMsg& cloud) const {
    const std::array<std::string, 6> required = {
        {"x", "y", "z", "intensity", "ring", "time"}};
    for (const auto& name : required) {
        const auto it = std::find_if(
            cloud.fields.begin(), cloud.fields.end(),
            [&name](const sensor_msgs::PointField& field) {
                return field.name == name;
            });
        if (it == cloud.fields.end()) {
            return false;
        }
    }
    return true;
}

bool MultiLidarFusion::transformCloudToOutputFrame(
    const CloudConstPtr& cloud, const ros::Time& output_stamp,
    PointCloud& transformed_cloud) {
    geometry_msgs::TransformStamped transform;
    try {
        if (!tf_buffer_.canTransform(config_.output_frame_id,
                                     cloud->header.frame_id,
                                     cloud->header.stamp,
                                     ros::Duration(config_.tf_timeout))) {
            ROS_WARN_STREAM("TF unavailable: " << cloud->header.frame_id
                                               << " -> "
                                               << config_.output_frame_id
                                               << " at "
                                               << cloud->header.stamp.toSec());
            return false;
        }
        transform = tf_buffer_.lookupTransform(
            config_.output_frame_id, cloud->header.frame_id,
            cloud->header.stamp, ros::Duration(config_.tf_timeout));
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM("TF lookup failed: " << e.what());
        return false;
    }

    PointCloud input_cloud;
    try {
        pcl::fromROSMsg(*cloud, input_cloud);
    } catch (const std::exception& e) {
        ++diagnostics_.dropped_conversion;
        ROS_WARN_STREAM("fromROSMsg failed for " << cloud->header.frame_id
                                                 << ": " << e.what());
        return false;
    }

    const Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
    transformed_cloud.clear();
    transformed_cloud.points.resize(input_cloud.points.size());
    transformed_cloud.width = input_cloud.width;
    transformed_cloud.height = input_cloud.height;
    transformed_cloud.is_dense = input_cloud.is_dense;
    transformed_cloud.header = input_cloud.header;

    const float time_offset =
        config_.normalize_point_time
            ? static_cast<float>((cloud->header.stamp - output_stamp).toSec())
            : 0.0f;

    for (size_t i = 0; i < input_cloud.points.size(); ++i) {
        const auto& src = input_cloud.points[i];
        auto& dst = transformed_cloud.points[i];
        const Eigen::Vector3f p(src.x, src.y, src.z);
        const Eigen::Vector3f q = transform_eigen.cast<float>() * p;
        dst.x = q.x();
        dst.y = q.y();
        dst.z = q.z();
        dst.intensity = src.intensity;
        dst.ring = src.ring;
        dst.time = src.time + time_offset;
    }

    return true;
}

bool MultiLidarFusion::fuseTransformedClouds(
    const std::vector<PointCloud::Ptr>& clouds, const ros::Time& output_stamp,
    CloudMsg& output_msg) {
    PointCloud::Ptr fused(new PointCloud);
    size_t total_points = 0;
    for (const auto& cloud : clouds) {
        total_points += cloud->points.size();
    }
    fused->points.reserve(total_points);
    fused->height = 1;
    fused->is_dense = false;

    for (const auto& cloud : clouds) {
        fused->points.insert(fused->points.end(), cloud->points.begin(),
                             cloud->points.end());
    }
    fused->width = static_cast<uint32_t>(fused->points.size());

    if (config_.enable_voxel_filter) {
        applyVoxelFilter(fused);
    }

    try {
        pcl::toROSMsg(*fused, output_msg);
    } catch (const std::exception& e) {
        ++diagnostics_.dropped_conversion;
        ROS_WARN_STREAM("toROSMsg failed: " << e.what());
        return false;
    }

    output_msg.header.stamp = output_stamp;
    output_msg.header.frame_id = config_.output_frame_id;
    return true;
}

void MultiLidarFusion::applyVoxelFilter(PointCloud::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return;
    }

    const double inv_leaf = 1.0 / config_.voxel_leaf_size;
    auto quantize = [inv_leaf](float v) -> int64_t {
        return static_cast<int64_t>(std::floor(static_cast<double>(v) *
                                              inv_leaf));
    };
    auto mix = [](int64_t x, int64_t y, int64_t z) -> uint64_t {
        uint64_t h = 1469598103934665603ull;
        auto add = [&h](int64_t value) {
            uint64_t v = static_cast<uint64_t>(value);
            h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
            h *= 1099511628211ull;
        };
        add(x);
        add(y);
        add(z);
        return h;
    };

    std::unordered_set<uint64_t> occupied;
    occupied.reserve(cloud->points.size());
    PointCloud::Ptr filtered(new PointCloud);
    filtered->points.reserve(cloud->points.size());
    filtered->height = 1;
    filtered->is_dense = cloud->is_dense;

    for (const auto& point : cloud->points) {
        const uint64_t key =
            mix(quantize(point.x), quantize(point.y), quantize(point.z));
        if (occupied.insert(key).second) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    cloud = filtered;
    ++diagnostics_.voxel_filtered;
}

ros::Time MultiLidarFusion::chooseOutputStamp(
    const std::vector<CloudConstPtr>& clouds) const {
    ros::Time output_stamp = clouds.front()->header.stamp;
    for (const auto& cloud : clouds) {
        if (cloud->header.stamp < output_stamp) {
            output_stamp = cloud->header.stamp;
        }
    }
    return output_stamp;
}

double MultiLidarFusion::computeSyncSpan(
    const std::vector<CloudConstPtr>& clouds) const {
    ros::Time min_stamp = clouds.front()->header.stamp;
    ros::Time max_stamp = clouds.front()->header.stamp;
    for (const auto& cloud : clouds) {
        min_stamp = std::min(min_stamp, cloud->header.stamp);
        max_stamp = std::max(max_stamp, cloud->header.stamp);
    }
    return (max_stamp - min_stamp).toSec();
}

void MultiLidarFusion::updateInputDiagnostics(
    const std::vector<CloudConstPtr>& clouds,
    const std::vector<size_t>& point_counts, double sync_span) {
    diagnostics_.last_sync_span = sync_span;
    diagnostics_.last_input_points = point_counts;
    diagnostics_.last_input_frames.resize(clouds.size());
    for (size_t i = 0; i < clouds.size(); ++i) {
        diagnostics_.last_input_frames[i] = clouds[i]->header.frame_id;
    }

    if (sync_span > config_.sync_slop * 0.8) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "Multi lidar sync span is near slop limit: "
                     << sync_span << " s, sync_slop=" << config_.sync_slop);
    }
}

void MultiLidarFusion::diagnosticsTimerCallback(const ros::TimerEvent&) {
    std::ostringstream counts;
    for (size_t i = 0; i < diagnostics_.last_input_points.size(); ++i) {
        if (i > 0) {
            counts << ", ";
        }
        counts << "lidar" << (i + 1) << "="
               << diagnostics_.last_input_points[i] << "pts/"
               << diagnostics_.last_input_frames[i];
    }

    ROS_INFO_STREAM("multi_lidar_fusion diagnostics: callbacks="
                    << diagnostics_.sync_callbacks
                    << ", published=" << diagnostics_.published
                    << ", last_output=" << diagnostics_.last_output_points
                    << "pts, last_sync_span=" << diagnostics_.last_sync_span
                    << "s, inputs=[" << counts.str() << "]"
                    << ", drops{empty=" << diagnostics_.dropped_empty
                    << ", min_points=" << diagnostics_.dropped_min_points
                    << ", field=" << diagnostics_.dropped_field
                    << ", tf=" << diagnostics_.dropped_tf
                    << ", conversion=" << diagnostics_.dropped_conversion
                    << ", exception=" << diagnostics_.dropped_exception
                    << "}");
}

}  // namespace lidar_fusion
