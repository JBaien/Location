#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <target_localizer/TargetMeasurement.h>
#include <target_localizer/TargetXY.h>
#include <visualization_msgs/Marker.h>

#include "target_localizer/cylinder_geometry.h"
#include "target_localizer/target_tracker.h"

namespace target_localizer {
namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

template <typename T>
std::string toString(const T& value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

diagnostic_msgs::KeyValue kv(const std::string& key,
                             const std::string& value) {
    diagnostic_msgs::KeyValue out;
    out.key = key;
    out.value = value;
    return out;
}

struct NodeConfig {
    std::string input_topic = "/points_raw";
    std::string target_frame = "base_link";
    std::string measurement_topic = "/target_measurement";
    std::string xy_topic = "/target_xy";
    std::string status_topic = "/target_status";
    std::string marker_topic = "/target_marker";
    std::string roi_topic = "/target_cloud_roi";
    double roi_x_min = -12.0;
    double roi_x_max = -4.0;
    double roi_y_min = -2.0;
    double roi_y_max = 2.0;
    double roi_z_min = -1.342;
    double roi_z_max = 0.858;
    double voxel_leaf = 0.02;
    double reflector_intensity_min = 180.0;
    int sor_mean_k = 20;
    double sor_stddev = 1.0;
    int normal_k = 20;
    int max_iterations = 200;
    double normal_distance_weight = 0.1;
    double cylinder_distance_threshold = 0.02;
    double radius_min = 0.095;
    double radius_max = 0.155;
    int min_candidate_points = 20;
    double reference_z = -0.142;
    double zero_x = -8.0;
    double zero_y = 0.0;
};

// target_localizer_node 是圆柱标靶定位链路的 ROS 封装：
//   /points_raw -> ROI 裁剪 -> 降采样/去离群 -> XY 平面圆 RANSAC -> 圆心
//               -> 常速度跟踪 -> /target_measurement + /target_xy + Marker/diagnostics
//
// 注意：本节点默认订阅的是三雷达融合后的 /points_raw，不再重复处理三路原始点云。
// 三雷达时间同步、TF 变换和点云字段保留由 lidar_fusion 包负责。
class TargetLocalizerNode {
public:
    TargetLocalizerNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          tracker_(loadTrackerConfig(private_nh)) {
        loadConfig();
        cloud_sub_ = nh_.subscribe(config_.input_topic, 2,
                                   &TargetLocalizerNode::cloudCallback, this);
        measurement_pub_ = nh_.advertise<TargetMeasurement>(
            config_.measurement_topic, 10, false);
        xy_pub_ = nh_.advertise<TargetXY>(config_.xy_topic, 10, false);
        status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>(
            config_.status_topic, 10, false);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            config_.marker_topic, 2, false);
        roi_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            config_.roi_topic, 2, false);
        diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
            "/diagnostics", 10, false);

        ROS_INFO_STREAM("target_localizer_node subscribed to "
                        << config_.input_topic << ", publishing "
                        << config_.xy_topic);
    }

private:
    static TrackerConfig loadTrackerConfig(ros::NodeHandle& private_nh) {
        TrackerConfig config;
        private_nh.param("min_good_inliers", config.min_good_inliers,
                         config.min_good_inliers);
        private_nh.param("min_update_inliers", config.min_update_inliers,
                         config.min_update_inliers);
        private_nh.param("good_residual_rms", config.good_residual_rms,
                         config.good_residual_rms);
        private_nh.param("max_update_residual_rms",
                         config.max_update_residual_rms,
                         config.max_update_residual_rms);
        private_nh.param("max_update_jump_m", config.max_update_jump_m,
                         config.max_update_jump_m);
        private_nh.param("max_velocity_mps", config.max_velocity_mps,
                         config.max_velocity_mps);
        private_nh.param("lost_after_misses", config.lost_after_misses,
                         config.lost_after_misses);
        private_nh.param("hold_duration", config.hold_duration,
                         config.hold_duration);
        return config;
    }

    void loadConfig() {
        // 所有现场可调参数都从节点私有命名空间读取，由 launch 中加载的 YAML 提供。
        // 这样 Docker 现场只需要修改 /config/target_localizer/target_localizer.yaml，
        // 不需要重新构建镜像。
        private_nh_.param("input_topic", config_.input_topic,
                          config_.input_topic);
        private_nh_.param("target_frame", config_.target_frame,
                          config_.target_frame);
        private_nh_.param("measurement_topic", config_.measurement_topic,
                          config_.measurement_topic);
        private_nh_.param("xy_topic", config_.xy_topic, config_.xy_topic);
        private_nh_.param("status_topic", config_.status_topic,
                          config_.status_topic);
        private_nh_.param("marker_topic", config_.marker_topic,
                          config_.marker_topic);
        private_nh_.param("roi_topic", config_.roi_topic, config_.roi_topic);
        private_nh_.param("roi_x_min", config_.roi_x_min, config_.roi_x_min);
        private_nh_.param("roi_x_max", config_.roi_x_max, config_.roi_x_max);
        private_nh_.param("roi_y_min", config_.roi_y_min, config_.roi_y_min);
        private_nh_.param("roi_y_max", config_.roi_y_max, config_.roi_y_max);
        private_nh_.param("roi_z_min", config_.roi_z_min, config_.roi_z_min);
        private_nh_.param("roi_z_max", config_.roi_z_max, config_.roi_z_max);
        private_nh_.param("voxel_leaf", config_.voxel_leaf,
                          config_.voxel_leaf);
        private_nh_.param("reflector_intensity_min",
                          config_.reflector_intensity_min,
                          config_.reflector_intensity_min);
        private_nh_.param("sor_mean_k", config_.sor_mean_k,
                          config_.sor_mean_k);
        private_nh_.param("sor_stddev", config_.sor_stddev,
                          config_.sor_stddev);
        private_nh_.param("normal_k", config_.normal_k, config_.normal_k);
        private_nh_.param("max_iterations", config_.max_iterations,
                          config_.max_iterations);
        private_nh_.param("normal_distance_weight",
                          config_.normal_distance_weight,
                          config_.normal_distance_weight);
        private_nh_.param("cylinder_distance_threshold",
                          config_.cylinder_distance_threshold,
                          config_.cylinder_distance_threshold);
        private_nh_.param("radius_min", config_.radius_min,
                          config_.radius_min);
        private_nh_.param("radius_max", config_.radius_max,
                          config_.radius_max);
        private_nh_.param("min_candidate_points", config_.min_candidate_points,
                          config_.min_candidate_points);
        private_nh_.param("reference_z", config_.reference_z,
                          config_.reference_z);
        private_nh_.param("zero_x", config_.zero_x, config_.zero_x);
        private_nh_.param("zero_y", config_.zero_y, config_.zero_y);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        ++frames_seen_;
        CloudT::Ptr raw(new CloudT);
        pcl::fromROSMsg(*msg, *raw);

        // 先做强先验 ROI 和反光强度筛选：标靶按设计安装在设备后方，
        // 且反光贴强度明显高于普通巷道结构，先提取候选反光点再拟合圆柱。
        CloudT::Ptr roi = cropRoi(*raw);
        publishRoi(*roi, msg->header);
        filterCloud(roi);

        CylinderModel model;
        bool detected = false;
        std::string reason;
        if (static_cast<int>(roi->size()) < config_.min_candidate_points) {
            reason = "not_enough_reflector_points";
        } else {
            detected = fitCylinder(*roi, model, reason);
        }

        if (!detected) {
            ++frames_missed_;
            // 检测失败不立即跳零。跟踪器会短时外推并根据连续丢帧次数降级到 LOST。
            const TrackerOutput out = tracker_.markMissed(msg->header.stamp.toSec());
            publishOutput(msg->header, out);
            publishDiagnostics(msg->header.stamp, out, 0, 0.0, reason);
            return;
        }

        // 业务只关心 XY 位移。拟合阶段已经把强反光点投影到 XY 平面做圆拟合，
        // 这里直接使用圆心，Z 固定为 reference_z 仅用于消息和 RViz marker 表达。
        const Eigen::Vector3d center = model.axis_point;
        Measurement measurement;
        measurement.stamp = msg->header.stamp.toSec();
        measurement.cx = center.x();
        measurement.cy = center.y();
        measurement.inlier_count = model.inlier_count;
        measurement.residual_rms = model.residual_rms;

        const TrackerOutput out = tracker_.update(measurement);
        publishMeasurement(msg->header, center, model, out);
        publishOutput(msg->header, out);
        publishMarker(msg->header, model);
        publishDiagnostics(msg->header.stamp, out, model.inlier_count,
                           model.residual_rms, "detected");
    }

    CloudT::Ptr cropRoi(const CloudT& input) const {
        CloudT::Ptr output(new CloudT);
        output->header = input.header;
        output->reserve(input.size());
        for (const PointT& point : input.points) {
            // 非有限点会破坏 PCL 滤波和法向估计，必须在最前面剔除。
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                continue;
            }
            if (point.x < config_.roi_x_min || point.x > config_.roi_x_max ||
                point.y < config_.roi_y_min || point.y > config_.roi_y_max ||
                point.z < config_.roi_z_min || point.z > config_.roi_z_max) {
                continue;
            }
            if (!std::isfinite(point.intensity) ||
                point.intensity <= config_.reflector_intensity_min) {
                continue;
            }
            output->push_back(point);
        }
        output->width = output->size();
        output->height = 1;
        output->is_dense = true;
        return output;
    }

    void filterCloud(CloudT::Ptr& cloud) const {
        // VoxelGrid 用 2 cm 默认体素压缩重复/密集点，保留局部几何质心。
        // 这里的 leaf 不能过大，否则 Ø250 mm 圆柱横向采样会被过度抹平。
        if (config_.voxel_leaf > 0.0 && !cloud->empty()) {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(config_.voxel_leaf, config_.voxel_leaf,
                              config_.voxel_leaf);
            CloudT::Ptr filtered(new CloudT);
            voxel.filter(*filtered);
            cloud = filtered;
        }
        // SOR 通过邻域统计剔除孤立噪点。粉尘场景下该步骤能减少 RANSAC 外点压力，
        // 但 mean_k 不能大于有效目标点数，否则小目标会被误伤。
        if (static_cast<int>(cloud->size()) > config_.sor_mean_k &&
            config_.sor_mean_k > 0) {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(config_.sor_mean_k);
            sor.setStddevMulThresh(config_.sor_stddev);
            CloudT::Ptr filtered(new CloudT);
            sor.filter(*filtered);
            cloud = filtered;
        }
    }

    bool fitCylinder(const CloudT& cloud, CylinderModel& model,
                     std::string& reason) const {
        if (cloud.size() < 3U) {
            reason = "not_enough_circle_points";
            return false;
        }

        const double target_radius =
            0.5 * (config_.radius_min + config_.radius_max);
        std::vector<double> center_x_candidates;
        std::vector<double> center_y_candidates;
        center_x_candidates.reserve(cloud.size());
        center_y_candidates.reserve(cloud.size());
        for (const PointT& point : cloud.points) {
            const double range_xy = std::hypot(point.x, point.y);
            if (range_xy <= 1e-6) {
                continue;
            }
            // 单雷达通常只能看到圆柱朝向雷达的一小段反光面，直接圆拟合会在
            // 多个合法圆之间跳变。把每个表面点沿雷达到该点的 XY 视线方向
            // 外推一个目标半径，可得到稳定的圆心候选。
            center_x_candidates.push_back(
                point.x + target_radius * point.x / range_xy);
            center_y_candidates.push_back(
                point.y + target_radius * point.y / range_xy);
        }
        if (center_x_candidates.size() < 3U) {
            reason = "not_enough_center_candidates";
            return false;
        }

        auto median = [](std::vector<double>& values) {
            const auto middle = values.begin() + values.size() / 2;
            std::nth_element(values.begin(), middle, values.end());
            return *middle;
        };
        const double cx = median(center_x_candidates);
        const double cy = median(center_y_candidates);
        if (!std::isfinite(cx) || !std::isfinite(cy)) {
            reason = "invalid_center_estimate";
            return false;
        }

        std::vector<int> inliers;
        inliers.reserve(cloud.size());
        double sum_sq = 0.0;
        for (int index = 0; index < static_cast<int>(cloud.size()); ++index) {
            const PointT& point = cloud.points[index];
            const double residual =
                std::abs(std::hypot(point.x - cx, point.y - cy) -
                         target_radius);
            if (residual <= config_.cylinder_distance_threshold) {
                inliers.push_back(index);
                sum_sq += residual * residual;
            }
        }
        if (inliers.size() < 3U) {
            reason = "not_enough_front_surface_inliers";
            return false;
        }

        model.axis_point = Eigen::Vector3d(cx, cy, config_.reference_z);
        model.axis_dir = Eigen::Vector3d::UnitZ();
        model.radius = target_radius;
        model.inlier_count = static_cast<int>(inliers.size());
        model.residual_rms = std::sqrt(
            sum_sq / std::max(1, model.inlier_count));
        return true;
    }

    void publishRoi(const CloudT& roi,
                    const std_msgs::Header& source_header) const {
        if (roi_pub_.getNumSubscribers() == 0) {
            return;
        }
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(roi, out);
        out.header = source_header;
        out.header.frame_id = config_.target_frame.empty()
                                  ? source_header.frame_id
                                  : config_.target_frame;
        roi_pub_.publish(out);
    }

    void publishMeasurement(const std_msgs::Header& header,
                            const Eigen::Vector3d& center,
                            const CylinderModel& model,
                            const TrackerOutput& out) const {
        TargetMeasurement msg;
        msg.header = header;
        msg.header.frame_id = config_.target_frame.empty()
                                  ? header.frame_id
                                  : config_.target_frame;
        msg.center_x = center.x();
        msg.center_y = center.y();
        msg.center_z = center.z();
        // 位移定义沿用 SLAM.md：建零点减当前目标中心。
        // 设备相对标靶移动时，目标在设备坐标系中的反向变化会被换算成设备位移。
        msg.dx = config_.zero_x - center.x();
        msg.dy = config_.zero_y - center.y();
        msg.radius = model.radius;
        msg.inlier_count = model.inlier_count;
        msg.residual_rms = model.residual_rms;
        const double variance = model.residual_rms * model.residual_rms;
        msg.covariance_xx = variance;
        msg.covariance_xy = 0.0;
        msg.covariance_yy = variance;
        msg.status = static_cast<std::uint8_t>(out.status);
        msg.status_text = statusText(out.status);
        measurement_pub_.publish(msg);
    }

    void publishOutput(const std_msgs::Header& header,
                       const TrackerOutput& out) const {
        TargetXY xy;
        xy.header = header;
        xy.header.frame_id = config_.target_frame.empty()
                                 ? header.frame_id
                                 : config_.target_frame;
        xy.center_x = out.cx;
        xy.center_y = out.cy;
        // /target_xy 是给上层控制/显示使用的轻量输出，只保留最终 XY 位移、
        // 当前中心点、速度估计和质量状态。
        xy.dx = config_.zero_x - out.cx;
        xy.dy = config_.zero_y - out.cy;
        xy.velocity_x = out.vx;
        xy.velocity_y = out.vy;
        xy.status = static_cast<std::uint8_t>(out.status);
        xy.status_text = statusText(out.status);
        xy_pub_.publish(xy);

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "target_localizer/status";
        status.hardware_id = "target_localizer";
        status.level = out.status == TargetStatus::GOOD
                           ? diagnostic_msgs::DiagnosticStatus::OK
                           : (out.status == TargetStatus::DEGRADED
                                  ? diagnostic_msgs::DiagnosticStatus::WARN
                                  : diagnostic_msgs::DiagnosticStatus::ERROR);
        status.message = statusText(out.status);
        status_pub_.publish(status);
    }

    void publishMarker(const std_msgs::Header& header,
                       const CylinderModel& model) const {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.header.frame_id = config_.target_frame.empty()
                                     ? header.frame_id
                                     : config_.target_frame;
        marker.ns = "target_localizer";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        // RViz Marker 用半透明圆柱显示检测结果，便于现场确认 ROI、半径和中心位置。
        // 第一阶段只显示竖直圆柱；轴线倾斜的精确姿态可后续再扩展。
        const Eigen::Vector3d center = model.axis_point;
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = config_.reference_z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = model.radius * 2.0;
        marker.scale.y = model.radius * 2.0;
        marker.scale.z = config_.roi_z_max - config_.roi_z_min;
        marker.color.r = 0.1f;
        marker.color.g = 0.9f;
        marker.color.b = 0.2f;
        marker.color.a = 0.45f;
        marker.lifetime = ros::Duration(0.3);
        marker_pub_.publish(marker);
    }

    void publishDiagnostics(const ros::Time& stamp,
                            const TrackerOutput& out,
                            int inliers,
                            double residual,
                            const std::string& reason) const {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = stamp;
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "target_localizer";
        status.hardware_id = "target_localizer";
        status.level = out.status == TargetStatus::GOOD
                           ? diagnostic_msgs::DiagnosticStatus::OK
                           : (out.status == TargetStatus::DEGRADED
                                  ? diagnostic_msgs::DiagnosticStatus::WARN
                                  : diagnostic_msgs::DiagnosticStatus::ERROR);
        status.message = reason;
        status.values.push_back(kv("frames_seen", toString(frames_seen_)));
        status.values.push_back(kv("frames_missed", toString(frames_missed_)));
        status.values.push_back(kv("status", statusText(out.status)));
        status.values.push_back(kv("inliers", toString(inliers)));
        status.values.push_back(kv("residual_rms", toString(residual)));
        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    NodeConfig config_;
    TargetTracker tracker_;
    ros::Subscriber cloud_sub_;
    ros::Publisher measurement_pub_;
    ros::Publisher xy_pub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher roi_pub_;
    ros::Publisher diagnostics_pub_;
    std::uint64_t frames_seen_ = 0;
    std::uint64_t frames_missed_ = 0;
};

}  // namespace
}  // namespace target_localizer

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_localizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    target_localizer::TargetLocalizerNode node(nh, private_nh);
    ros::spin();
    return 0;
}
