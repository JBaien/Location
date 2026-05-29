#include <atomic>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <target_localizer/EquipmentState.h>
#include <target_localizer/TargetXY.h>

#include "mine_slam_web/pointcloud_encoder.h"
#include "mine_slam_web/websocket_server.h"

namespace mine_slam_web {
namespace {

struct PoseState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  bool valid = false;
};

struct MapTransformSnapshot {
  std::array<double, 9> rotation{1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0};
  std::array<double, 3> translation{0.0, 0.0, 0.0};
  bool valid = false;
};

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  ros::Time stamp;
};

struct ProgressiveRevealState {
  bool seen = false;
  bool enabled = false;
  double machine_s = 0.0;
  double revealed_s = 0.0;
  double face_wall_s = 0.0;
  int visible_reflector_count = 0;
  int hidden_reflector_count = 0;
  int hidden_unrevealed_point_count = 0;
  int filter_front_unrevealed_point_count = 0;
  int published_face_point_count = 0;
  std::string reveal_source = "unknown";
};

struct EquipmentStateSnapshot {
  bool seen = false;
  bool attitude_valid = false;
  bool distances_valid = false;
  bool roll_valid = false;
  bool pitch_valid = false;
  bool yaw_valid = false;
  bool left_front_valid = false;
  bool left_rear_valid = false;
  bool right_front_valid = false;
  bool right_rear_valid = false;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double ground_plane_rmse = 0.0;
  double left_front_mm = 0.0;
  double left_rear_mm = 0.0;
  double right_front_mm = 0.0;
  double right_rear_mm = 0.0;
  int ground_points = 0;
  int wall_points = 0;
  int left_front_points = 0;
  int left_rear_points = 0;
  int right_front_points = 0;
  int right_rear_points = 0;
  int point_count = 0;
  std::string quality = "lost";
  std::string source = "none";
  std::uint64_t stamp_ns = 0;
};

struct TargetXySnapshot {
  bool seen = false;
  double center_x_mm = 0.0;
  double center_y_mm = 0.0;
  double dx_mm = 0.0;
  double dy_mm = 0.0;
  double velocity_x_mm_s = 0.0;
  double velocity_y_mm_s = 0.0;
  int status = 0;
  std::string status_text = "lost";
  std::uint64_t stamp_ns = 0;
};

enum class OdomSource {
  NONE,
  PREFERRED,
  FALLBACK,
};

double distance(const PathPoint& a, const PathPoint& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

MapTransformSnapshot poseToMapTransform(const PoseState& pose) {
  MapTransformSnapshot transform;
  if (!pose.valid) {
    return transform;
  }
  const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy +
                                pose.qz * pose.qz + pose.qw * pose.qw);
  if (norm <= 1e-12) {
    return transform;
  }
  const double x = pose.qx / norm;
  const double y = pose.qy / norm;
  const double z = pose.qz / norm;
  const double w = pose.qw / norm;
  transform.rotation = {
      1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
      2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
      2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)};
  transform.translation = {pose.x, pose.y, pose.z};
  transform.valid = true;
  return transform;
}

class MineWebBridgeNode {
 public:
  MineWebBridgeNode() : nh_(), cloud_server_(readPort("cloud_port", 9001), true), status_server_(readPort("status_port", 9002), false) {
    loadConfig();
    cloud_server_.start();
    status_server_.start();
    cloud_server_.setClientConnectedCallback([this]() { sendInitialSnapshots("cloud_client_initial_snapshot"); });
    status_server_.setClientConnectedCallback([this]() { sendInitialSnapshots("status_client_initial_snapshot"); });

    current_sub_ = nh_.subscribe(current_cloud_topic_, 1, &MineWebBridgeNode::currentCloudCallback, this);
    stable_sub_ = nh_.subscribe(stable_map_topic_, 1, &MineWebBridgeNode::stableMapCallback, this);
    if (!path_topic_.empty()) {
      path_sub_ = nh_.subscribe(path_topic_, 5, &MineWebBridgeNode::pathCallback, this);
    }
    if (!progressive_reveal_topic_.empty()) {
      progressive_reveal_sub_ =
          nh_.subscribe(progressive_reveal_topic_, 10, &MineWebBridgeNode::progressiveRevealCallback, this);
    }
    if (!equipment_state_topic_.empty()) {
      equipment_state_sub_ =
          nh_.subscribe(equipment_state_topic_, 20, &MineWebBridgeNode::equipmentStateCallback, this);
    }
    if (!sensor_reference_topic_.empty()) {
      sensor_reference_sub_ =
          nh_.subscribe(sensor_reference_topic_, 20, &MineWebBridgeNode::sensorReferenceCallback, this);
    }
    if (!target_xy_topic_.empty()) {
      target_xy_sub_ =
          nh_.subscribe(target_xy_topic_, 20, &MineWebBridgeNode::targetXyCallback, this);
    }
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &MineWebBridgeNode::preferredOdomCallback, this);
    odom_fallback_sub_ = nh_.subscribe(odom_fallback_topic_, 50, &MineWebBridgeNode::fallbackOdomCallback, this);
    clock_sub_ = nh_.subscribe("/clock", 10, &MineWebBridgeNode::clockCallback, this);
    status_timer_ = nh_.createTimer(ros::Duration(0.1), &MineWebBridgeNode::statusTimerCallback, this);

    ROS_INFO_STREAM("mine_slam_web bridge listening: cloud ws port " << cloud_port_
                    << ", status ws port " << status_port_);
    ROS_INFO_STREAM("mine_slam_web topics: current=" << current_cloud_topic_
                    << ", stable=" << stable_map_topic_
                    << ", path=" << path_topic_
                    << ", progressive=" << progressive_reveal_topic_
                    << ", equipment=" << equipment_state_topic_
                    << ", reference=" << sensor_reference_topic_
                    << ", target_xy=" << target_xy_topic_
                    << ", odom=" << odom_topic_
                    << ", fallback=" << odom_fallback_topic_);
  }

 private:
  static int readPort(const std::string& name, int fallback) {
    int value = fallback;
    ros::param::param("web_viewer/websocket/" + name, value, fallback);
    return value;
  }

  void loadConfig() {
    cloud_port_ = readPort("cloud_port", 9001);
    status_port_ = readPort("status_port", 9002);
    nh_.param<std::string>("web_viewer/topics/current_cloud", current_cloud_topic_, "/cloud_registered_body");
    nh_.param<std::string>("web_viewer/topics/stable_map", stable_map_topic_, "/slam/stable_map");
    nh_.param<std::string>("web_viewer/topics/path", path_topic_, "/slam/global_path");
    nh_.param<std::string>("web_viewer/topics/progressive_reveal_diagnostics",
                           progressive_reveal_topic_, "/sim/progressive_reveal_diagnostics");
    nh_.param<std::string>("web_viewer/topics/equipment_state",
                           equipment_state_topic_, "/equipment_state");
    nh_.param<std::string>("web_viewer/topics/sensor_reference",
                           sensor_reference_topic_, "/sensor_reference");
    nh_.param<std::string>("web_viewer/topics/target_xy",
                           target_xy_topic_, "/target_xy");
    nh_.param<std::string>("web_viewer/topics/odom", odom_topic_, "/slam/global_odom");
    nh_.param<std::string>("web_viewer/topics/odom_fallback", odom_fallback_topic_, "/Odometry");
    nh_.param<double>("web_viewer/current_cloud/send_rate_hz", current_rate_hz_, 2.0);
    nh_.param<double>("web_viewer/current_cloud/voxel_size_m", current_voxel_size_m_, 0.15);
    nh_.param<int>("web_viewer/current_cloud/max_points", current_max_points_, 100000);
    nh_.param<bool>("web_viewer/current_cloud/transform_body_to_map",
                    transform_current_body_to_map_, true);
    nh_.param<double>("web_viewer/stable_map/send_rate_hz", stable_rate_hz_, 0.5);
    nh_.param<double>("web_viewer/stable_map/voxel_size_m", stable_voxel_size_m_, 0.20);
    nh_.param<int>("web_viewer/stable_map/max_points", stable_max_points_, 300000);
    nh_.param<int>("web_viewer/path/max_points", path_max_points_, 5000);
    nh_.param<double>("web_viewer/path/min_step_m", path_min_step_m_, 0.05);
    nh_.param<double>("web_viewer/path/reset_jump_m", path_reset_jump_m_, 5.0);
    nh_.param<double>("web_viewer/reflector/intensity_threshold", reflector_intensity_threshold_, 180.0);
  }

  void currentCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    handleCloud(*msg, CLOUD_CURRENT, current_rate_hz_, current_voxel_size_m_,
                static_cast<std::size_t>(std::max(0, current_max_points_)), last_current_send_);
  }

  void stableMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    handleCloud(*msg, CLOUD_STABLE, stable_rate_hz_, stable_voxel_size_m_,
                static_cast<std::size_t>(std::max(0, stable_max_points_)), last_stable_send_);
  }

  void pathCallback(const nav_msgs::PathConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (messageAheadOfClockLocked(msg->header.stamp)) {
      return;
    }
    const std::string next_frame = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    if (path_snapshot_seen_ && !path_snapshot_frame_.empty() && path_snapshot_frame_ != next_frame) {
      last_path_reset_reason_ = "frame_changed";
      ++path_reset_count_;
    } else if (path_snapshot_seen_ && !path_snapshot_stamp_.isZero() && !msg->header.stamp.isZero() &&
               msg->header.stamp + ros::Duration(0.1) < path_snapshot_stamp_) {
      last_path_reset_reason_ = "stamp_regression_ignored_for_full_snapshot";
    }
    path_snapshot_.clear();
    path_snapshot_.reserve(std::min<std::size_t>(
        msg->poses.size(), static_cast<std::size_t>(std::max(1, path_max_points_))));
    const std::size_t stride =
        path_max_points_ > 0 && msg->poses.size() > static_cast<std::size_t>(path_max_points_)
            ? static_cast<std::size_t>(
                  std::ceil(static_cast<double>(msg->poses.size()) /
                            static_cast<double>(path_max_points_)))
            : 1U;
    std::size_t last_added = msg->poses.size();
    for (std::size_t i = 0; i < msg->poses.size(); i += stride) {
      const auto& pose = msg->poses[i];
      const auto& p = pose.pose.position;
      path_snapshot_.push_back(PathPoint{p.x, p.y, p.z, pose.header.stamp});
      last_added = i;
    }
    if (!msg->poses.empty() && last_added + 1U != msg->poses.size()) {
      const auto& pose = msg->poses.back();
      const auto& p = pose.pose.position;
      path_snapshot_.push_back(PathPoint{p.x, p.y, p.z, pose.header.stamp});
    }
    path_snapshot_frame_ = next_frame;
    path_snapshot_stamp_ = msg->header.stamp;
    path_snapshot_seen_ = true;
  }

  void handleCloud(const sensor_msgs::PointCloud2& msg,
                   CloudType type,
                   double send_rate_hz,
                   double voxel_size_m,
                   std::size_t max_points,
                   ros::Time& last_send) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (messageAheadOfClockLocked(msg.header.stamp)) {
        return;
      }
    }
    const std::size_t clients = cloud_server_.clientCount();
    if (clients == 0) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const double min_period = send_rate_hz > 0.0 ? 1.0 / send_rate_hz : 0.0;
    if (!last_send.isZero() && (now - last_send).toSec() < min_period) {
      return;
    }
    last_send = now;

    CloudEncodeOptions options;
    options.cloud_type = type;
    options.max_points = max_points;
    options.voxel_size_m = voxel_size_m;
    options.reflector_intensity_threshold = static_cast<float>(reflector_intensity_threshold_);
    if (type == CLOUD_CURRENT && transform_current_body_to_map_) {
      MapTransformSnapshot transform;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        transform = poseToMapTransform(pose_);
      }
      if (!transform.valid) {
        ROS_WARN_STREAM_THROTTLE(
            2.0,
            "mine_slam_web: skip current body cloud until a valid map odom pose is available");
        return;
      }
      options.transform_to_map = true;
      std::copy(transform.rotation.begin(), transform.rotation.end(), options.rotation);
      std::copy(transform.translation.begin(), transform.translation.end(), options.translation);
    }
    auto encoded = encodePointCloud2(msg, options);

    if (type == CLOUD_CURRENT) {
      current_raw_points_.store(encoded.raw_points);
      current_encoded_points_.store(encoded.encoded_points);
    } else {
      stable_raw_points_.store(encoded.raw_points);
      stable_encoded_points_.store(encoded.encoded_points);
    }

    if (!encoded.buffer.empty()) {
      auto packet = std::make_shared<const std::vector<std::uint8_t>>(std::move(encoded.buffer));
      {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        if (type == CLOUD_CURRENT) {
          latest_current_cloud_packet_ = packet;
          latest_current_stamp_ = msg.header.stamp;
        } else {
          latest_stable_map_packet_ = packet;
          latest_stable_stamp_ = msg.header.stamp;
        }
      }
      cloud_server_.broadcast(packet);
    }
  }

  void preferredOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    preferred_odom_seen_ = true;
    preferred_odom_seen_until_ = ros::Time::now() + ros::Duration(2.0);
    handleOdom(*msg, OdomSource::PREFERRED);
  }

  void fallbackOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    // The fallback is only for stacks that never publish /slam/global_odom.
    // Mixing fallback /Odometry history with later map-frame global odom creates
    // long false trajectory chords in the viewer.
    if (preferred_odom_seen_ || ros::Time::now() < preferred_odom_seen_until_) {
      return;
    }
    handleOdom(*msg, OdomSource::FALLBACK);
  }

  static bool jsonBool(const std::string& text, const std::string& key, bool fallback = false) {
    const std::string needle = "\"" + key + "\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto start = pos + needle.size();
    if (text.compare(start, 4, "true") == 0) {
      return true;
    }
    if (text.compare(start, 5, "false") == 0) {
      return false;
    }
    return fallback;
  }

  static double jsonDouble(const std::string& text, const std::string& key, double fallback = 0.0) {
    const std::string needle = "\"" + key + "\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto start = pos + needle.size();
    char* end = nullptr;
    const double value = std::strtod(text.c_str() + start, &end);
    return end == text.c_str() + start ? fallback : value;
  }

  static int jsonInt(const std::string& text, const std::string& key, int fallback = 0) {
    return static_cast<int>(std::lround(jsonDouble(text, key, static_cast<double>(fallback))));
  }

  static std::string jsonString(const std::string& text, const std::string& key, const std::string& fallback) {
    const std::string needle = "\"" + key + "\":\"";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto start = pos + needle.size();
    const auto end = text.find('"', start);
    return end == std::string::npos ? fallback : text.substr(start, end - start);
  }

  void progressiveRevealCallback(const std_msgs::StringConstPtr& msg) {
    ProgressiveRevealState state;
    state.seen = true;
    state.enabled = jsonBool(msg->data, "progressive_reveal_enabled", false);
    state.machine_s = jsonDouble(msg->data, "machine_s", 0.0);
    state.revealed_s = jsonDouble(msg->data, "revealed_s", 0.0);
    state.face_wall_s = jsonDouble(msg->data, "face_wall_s", 0.0);
    state.visible_reflector_count = jsonInt(msg->data, "visible_reflector_count", 0);
    state.hidden_reflector_count = jsonInt(msg->data, "hidden_reflector_count", 0);
    state.hidden_unrevealed_point_count = jsonInt(msg->data, "hidden_unrevealed_point_count", 0);
    state.filter_front_unrevealed_point_count = jsonInt(msg->data, "filter_front_unrevealed_point_count", 0);
    state.published_face_point_count = jsonInt(msg->data, "published_face_point_count", 0);
    state.reveal_source = jsonString(msg->data, "reveal_source", "unknown");
    std::lock_guard<std::mutex> lock(state_mutex_);
    progressive_reveal_ = state;
  }

  static EquipmentStateSnapshot snapshotFromEquipmentMsg(const target_localizer::EquipmentState& msg) {
    EquipmentStateSnapshot snapshot;
    snapshot.seen = true;
    snapshot.attitude_valid = msg.attitude_valid;
    snapshot.distances_valid = msg.distances_valid;
    snapshot.roll_valid = msg.roll_valid;
    snapshot.pitch_valid = msg.pitch_valid;
    snapshot.yaw_valid = msg.yaw_valid;
    snapshot.left_front_valid = msg.left_front_valid;
    snapshot.left_rear_valid = msg.left_rear_valid;
    snapshot.right_front_valid = msg.right_front_valid;
    snapshot.right_rear_valid = msg.right_rear_valid;
    snapshot.roll_deg = msg.roll_deg;
    snapshot.pitch_deg = msg.pitch_deg;
    snapshot.yaw_deg = msg.yaw_deg;
    snapshot.ground_plane_rmse = msg.ground_plane_rmse;
    snapshot.left_front_mm = msg.left_front_mm;
    snapshot.left_rear_mm = msg.left_rear_mm;
    snapshot.right_front_mm = msg.right_front_mm;
    snapshot.right_rear_mm = msg.right_rear_mm;
    snapshot.ground_points = static_cast<int>(msg.ground_points);
    snapshot.wall_points = static_cast<int>(msg.wall_points);
    snapshot.left_front_points = static_cast<int>(msg.left_front_points);
    snapshot.left_rear_points = static_cast<int>(msg.left_rear_points);
    snapshot.right_front_points = static_cast<int>(msg.right_front_points);
    snapshot.right_rear_points = static_cast<int>(msg.right_rear_points);
    snapshot.point_count = static_cast<int>(msg.point_count);
    snapshot.quality = msg.quality;
    snapshot.source = msg.source;
    snapshot.stamp_ns = stampNs(msg.header.stamp);
    return snapshot;
  }

  void equipmentStateCallback(const target_localizer::EquipmentStateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cloud_equipment_ = snapshotFromEquipmentMsg(*msg);
  }

  void sensorReferenceCallback(const target_localizer::EquipmentStateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sensor_reference_ = snapshotFromEquipmentMsg(*msg);
  }

  void targetXyCallback(const target_localizer::TargetXYConstPtr& msg) {
    TargetXySnapshot snapshot;
    snapshot.seen = true;
    snapshot.center_x_mm = msg->center_x * 1000.0;
    snapshot.center_y_mm = msg->center_y * 1000.0;
    snapshot.dx_mm = msg->dx * 1000.0;
    snapshot.dy_mm = msg->dy * 1000.0;
    snapshot.velocity_x_mm_s = msg->velocity_x * 1000.0;
    snapshot.velocity_y_mm_s = msg->velocity_y * 1000.0;
    snapshot.status = static_cast<int>(msg->status);
    snapshot.status_text = msg->status_text;
    snapshot.stamp_ns = stampNs(msg->header.stamp);
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_xy_ = snapshot;
  }

  void clockCallback(const rosgraph_msgs::ClockConstPtr& msg) {
    bool reset = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      reset = !last_clock_stamp_.isZero() && !msg->clock.isZero() &&
              msg->clock + ros::Duration(0.5) < last_clock_stamp_;
      current_clock_stamp_ = msg->clock;
      last_clock_stamp_ = msg->clock;
    }
    if (reset) {
      clearRunState("new_session");
    }
  }

  void handleOdom(const nav_msgs::Odometry& msg, OdomSource source) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (messageAheadOfClockLocked(msg.header.stamp)) {
      return;
    }
    const std::string frame_id = msg.header.frame_id.empty() ? "map" : msg.header.frame_id;
    const bool source_changed = odom_source_ != OdomSource::NONE && odom_source_ != source;
    const bool frame_changed = !path_frame_.empty() && path_frame_ != frame_id;
    const bool stamp_regressed =
        !last_odom_path_stamp_.isZero() && !msg.header.stamp.isZero() &&
        msg.header.stamp + ros::Duration(0.1) < last_odom_path_stamp_;
    if (path_snapshot_seen_) {
      if (source_changed) {
        last_path_reset_reason_ = "source_changed_ignored_for_full_snapshot";
      } else if (frame_changed) {
        last_path_reset_reason_ = "frame_changed_ignored_for_full_snapshot";
      } else if (stamp_regressed) {
        last_path_reset_reason_ = "stamp_regression_ignored_for_full_snapshot";
      }
    } else if (source_changed || frame_changed) {
      resetPathLocked(frame_id, source, source_changed ? "source_changed" : "frame_changed");
    } else if (stamp_regressed) {
      resetPathLocked(frame_id, source, "delta_stamp_regression");
    }
    odom_source_ = source;
    path_frame_ = frame_id;
    if (!msg.header.stamp.isZero()) {
      last_odom_path_stamp_ = msg.header.stamp;
    }

    pose_.x = msg.pose.pose.position.x;
    pose_.y = msg.pose.pose.position.y;
    pose_.z = msg.pose.pose.position.z;
    pose_.qx = msg.pose.pose.orientation.x;
    pose_.qy = msg.pose.pose.orientation.y;
    pose_.qz = msg.pose.pose.orientation.z;
    pose_.qw = msg.pose.pose.orientation.w;
    pose_.valid = true;
    map_frame_ = frame_id;

    PathPoint next{pose_.x, pose_.y, pose_.z, msg.header.stamp};
    if (!path_snapshot_seen_ && !path_.empty() && path_reset_jump_m_ > 0.0 &&
        distance(path_.back(), next) > path_reset_jump_m_) {
      resetPathLocked(frame_id, source, "odom_jump_reset");
    }
    if (path_.empty() || distance(path_.back(), next) >= path_min_step_m_) {
      path_.push_back(next);
      if (path_max_points_ > 0 && path_.size() > static_cast<std::size_t>(path_max_points_)) {
        path_.erase(path_.begin(), path_.begin() + static_cast<long>(path_.size() - path_max_points_));
      }
    }
  }

  void resetPathLocked(const std::string& frame_id, OdomSource source, const std::string& reason) {
    path_.clear();
    path_frame_ = frame_id;
    odom_source_ = source;
    last_path_reset_reason_ = reason;
    ++path_reset_count_;
  }

  bool messageAheadOfClockLocked(const ros::Time& stamp) const {
    if (stamp.isZero() || current_clock_stamp_.isZero()) {
      return false;
    }
    return stamp > current_clock_stamp_ + ros::Duration(5.0);
  }

  std::shared_ptr<const std::vector<std::uint8_t>> makeEmptyCloudPacket(CloudType type, const ros::Time& stamp) const {
    CloudPacketHeader header{};
    header.magic = kCloudPacketMagic;
    header.version = kCloudPacketVersion;
    header.cloud_type = static_cast<std::uint16_t>(type);
    header.stamp_ns = static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
                      static_cast<std::uint64_t>(stamp.nsec);
    header.point_count = 0;
    header.fields_mask = 0;
    auto bytes = std::make_shared<std::vector<std::uint8_t>>(sizeof(CloudPacketHeader));
    std::memcpy(bytes->data(), &header, sizeof(header));
    return bytes;
  }

  void clearRunState(const std::string& reason) {
    const ros::Time stamp = ros::Time::now();
    auto empty_current = makeEmptyCloudPacket(CLOUD_CURRENT, stamp);
    auto empty_stable = makeEmptyCloudPacket(CLOUD_STABLE, stamp);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      path_.clear();
      path_snapshot_.clear();
      path_snapshot_seen_ = false;
      path_snapshot_stamp_ = ros::Time();
      last_odom_path_stamp_ = ros::Time();
      preferred_odom_seen_ = false;
      preferred_odom_seen_until_ = ros::Time();
      odom_source_ = OdomSource::NONE;
      last_path_reset_reason_ = reason;
      ++path_reset_count_;
    }
    current_raw_points_.store(0);
    current_encoded_points_.store(0);
    stable_raw_points_.store(0);
    stable_encoded_points_.store(0);
    last_current_send_ = ros::Time();
    last_stable_send_ = ros::Time();
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_current_cloud_packet_ = empty_current;
      latest_stable_map_packet_ = empty_stable;
      latest_initial_snapshot_reason_ = reason;
      ++session_id_;
    }
    cloud_server_.broadcast(empty_stable);
    cloud_server_.broadcast(empty_current);
    const std::string status = buildStatusJson();
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_status_json_ = status;
    }
    status_server_.broadcastText(status);
  }

  const char* odomSourceName(OdomSource source) const {
    switch (source) {
      case OdomSource::PREFERRED:
        return "preferred";
      case OdomSource::FALLBACK:
        return "fallback";
      case OdomSource::NONE:
      default:
        return "none";
    }
  }

  static double pathSpanX(const std::vector<PathPoint>& path) {
    if (path.empty()) {
      return 0.0;
    }
    double min_x = path.front().x;
    double max_x = path.front().x;
    for (const auto& point : path) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }
    return std::max(0.0, max_x - min_x);
  }

  static double firstX(const std::vector<PathPoint>& path) {
    return path.empty() ? 0.0 : path.front().x;
  }

  static double lastX(const std::vector<PathPoint>& path) {
    return path.empty() ? 0.0 : path.back().x;
  }

  static std::size_t statusPathStride(std::size_t point_count) {
    constexpr std::size_t kMaxStatusPathPoints = 800U;
    if (point_count <= kMaxStatusPathPoints) {
      return 1U;
    }
    return static_cast<std::size_t>(
        std::ceil(static_cast<double>(point_count) / static_cast<double>(kMaxStatusPathPoints)));
  }

  static std::uint64_t stampNs(const ros::Time& stamp) {
    return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
           static_cast<std::uint64_t>(stamp.nsec);
  }

  bool snapshotPathLooksCompleteLocked() const {
    if (!path_snapshot_seen_ || path_snapshot_.empty()) {
      return false;
    }
    if (path_.size() < 5) {
      return true;
    }
    const double snapshot_span = pathSpanX(path_snapshot_);
    const double odom_span = pathSpanX(path_);
    const double start_gap = std::abs(firstX(path_snapshot_) - firstX(path_));
    return start_gap <= 2.0 && (odom_span < 5.0 || snapshot_span >= odom_span * 0.70);
  }

  std::string buildStatusJson() {
    PoseState pose;
    std::vector<PathPoint> path;
    std::string map_frame;
    std::string path_frame;
    std::string path_source;
    std::string last_path_reset_reason;
    std::uint64_t path_snapshot_stamp_ns = 0;
    std::uint64_t path_start_stamp_ns = 0;
    std::uint64_t path_end_stamp_ns = 0;
    OdomSource odom_source = OdomSource::NONE;
    std::uint64_t path_reset_count = 0;
    std::size_t odom_path_count = 0;
    std::size_t snapshot_path_count = 0;
    double path_start_s = 0.0;
    double path_end_s = 0.0;
    double path_snapshot_span_s = 0.0;
    double path_odom_span_s = 0.0;
    bool path_snapshot_complete = false;
    ProgressiveRevealState progressive;
    EquipmentStateSnapshot cloud_equipment;
    EquipmentStateSnapshot sensor_reference;
    TargetXySnapshot target_xy;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose = pose_;
      odom_path_count = path_.size();
      snapshot_path_count = path_snapshot_.size();
      path_snapshot_span_s = pathSpanX(path_snapshot_);
      path_odom_span_s = pathSpanX(path_);
      path_snapshot_complete = snapshotPathLooksCompleteLocked();
      if (path_snapshot_seen_ && !path_snapshot_.empty() && path_snapshot_complete) {
        path = path_snapshot_;
        path_frame = path_snapshot_frame_;
        path_source = path_topic_.empty() ? "path_snapshot" : path_topic_;
        path_snapshot_stamp_ns = stampNs(path_snapshot_stamp_);
      } else {
        path = path_;
        path_frame = path_frame_;
        path_source = odom_source_ == OdomSource::FALLBACK
                          ? "bridge_accumulated_fallback_odom"
                          : "bridge_accumulated_global_odom";
      }
      path_start_s = firstX(path);
      path_end_s = lastX(path);
      if (!path.empty()) {
        path_start_stamp_ns = stampNs(path.front().stamp);
        path_end_stamp_ns = stampNs(path.back().stamp);
      }
      map_frame = map_frame_;
      odom_source = odom_source_;
      path_reset_count = path_reset_count_;
      last_path_reset_reason = last_path_reset_reason_;
      progressive = progressive_reveal_;
      cloud_equipment = cloud_equipment_;
      sensor_reference = sensor_reference_;
      target_xy = target_xy_;
    }

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(4);
    json << "{";
    json << "\"connected\":true,";
    json << "\"cloud_clients\":" << cloud_server_.clientCount() << ",";
    json << "\"status_clients\":" << status_server_.clientCount() << ",";
    json << "\"current_cloud_points\":" << current_encoded_points_.load() << ",";
    json << "\"current_cloud_raw_points\":" << current_raw_points_.load() << ",";
    json << "\"stable_map_points\":" << stable_encoded_points_.load() << ",";
    json << "\"stable_map_raw_points\":" << stable_raw_points_.load() << ",";
    json << "\"latest_current_cloud_cached\":" << (hasLatestCurrentCloud() ? "true" : "false") << ",";
    json << "\"latest_stable_map_cached\":" << (hasLatestStableMap() ? "true" : "false") << ",";
    json << "\"latest_path_cached\":" << ((snapshot_path_count > 0) ? "true" : "false") << ",";
    json << "\"initial_snapshot_reason\":\"" << latest_initialSnapshotReason() << "\",";
    json << "\"session_id\":" << sessionId() << ",";
    json << "\"current_cloud_source_topic\":\"" << current_cloud_topic_ << "\",";
    json << "\"stable_map_source_topic\":\"" << stable_map_topic_ << "\",";
    json << "\"current_layer_semantics\":\"replace\",";
    json << "\"stable_layer_semantics\":\"replace\",";
    json << "\"pose\":{";
    json << "\"valid\":" << (pose.valid ? "true" : "false") << ",";
    json << "\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"z\":" << pose.z << ",";
    json << "\"qx\":" << pose.qx << ",\"qy\":" << pose.qy << ",\"qz\":" << pose.qz << ",\"qw\":" << pose.qw;
    json << "},";
    const std::size_t path_stride = statusPathStride(path.size());
    std::size_t emitted_path_points = 0U;
    json << "\"path\":[";
    for (std::size_t i = 0; i < path.size(); i += path_stride) {
      if (emitted_path_points > 0) {
        json << ",";
      }
      json << "[" << path[i].x << "," << path[i].y << "," << path[i].z << "]";
      ++emitted_path_points;
    }
    if (!path.empty() && (emitted_path_points == 0U || ((path.size() - 1U) % path_stride) != 0U)) {
      if (emitted_path_points > 0) {
        json << ",";
      }
      const auto& tail = path.back();
      json << "[" << tail.x << "," << tail.y << "," << tail.z << "]";
      ++emitted_path_points;
    }
    json << "],";
    json << "\"map_frame\":\"" << map_frame << "\",";
    json << "\"path_frame\":\"" << path_frame << "\",";
    json << "\"path_source\":\"" << path_source << "\",";
    json << "\"full_path_point_count\":" << snapshot_path_count << ",";
    json << "\"odom_path_point_count\":" << odom_path_count << ",";
    json << "\"displayed_path_point_count\":" << path.size() << ",";
    json << "\"status_path_point_count\":" << emitted_path_points << ",";
    json << "\"status_path_stride\":" << path_stride << ",";
    json << "\"max_path_points\":" << path_max_points_ << ",";
    json << "\"path_is_snapshot_or_delta\":\""
         << ((path_snapshot_seen_ && !path_snapshot_.empty() && path_snapshot_complete) ? "full_snapshot" : "odom_accumulated")
         << "\",";
    json << "\"path_snapshot_stamp_ns\":" << path_snapshot_stamp_ns << ",";
    json << "\"path_start_stamp_ns\":" << path_start_stamp_ns << ",";
    json << "\"path_end_stamp_ns\":" << path_end_stamp_ns << ",";
    json << "\"path_start_s\":" << path_start_s << ",";
    json << "\"path_end_s\":" << path_end_s << ",";
    json << "\"path_snapshot_span_s\":" << path_snapshot_span_s << ",";
    json << "\"path_odom_span_s\":" << path_odom_span_s << ",";
    json << "\"path_snapshot_complete\":" << (path_snapshot_complete ? "true" : "false") << ",";
    json << "\"odom_source\":\"" << odomSourceName(odom_source) << "\",";
    json << "\"path_reset_count\":" << path_reset_count << ",";
    json << "\"last_path_reset_reason\":\"" << last_path_reset_reason << "\",";
    json << "\"fixed_frame\":\"map\",";
    json << "\"pointcloud_frame\":\"map\",";
    json << "\"pose_frame\":\"" << map_frame << "\",";
    json << "\"transform_applied_in_backend\":true,";
    json << "\"transform_applied_in_frontend\":false,";
    json << "\"transform_axis_mapping\":\"ros_xyz_to_three_xyz\",";
    json << "\"double_transform_detected\":false,";
    json << "\"progressive_reveal_seen\":" << (progressive.seen ? "true" : "false") << ",";
    json << "\"progressive_reveal_enabled\":" << (progressive.enabled ? "true" : "false") << ",";
    json << "\"progressive_machine_s\":" << progressive.machine_s << ",";
    json << "\"progressive_revealed_s\":" << progressive.revealed_s << ",";
    json << "\"progressive_face_wall_s\":" << progressive.face_wall_s << ",";
    json << "\"progressive_visible_reflector_count\":" << progressive.visible_reflector_count << ",";
    json << "\"progressive_hidden_reflector_count\":" << progressive.hidden_reflector_count << ",";
    json << "\"progressive_hidden_unrevealed_point_count\":" << progressive.hidden_unrevealed_point_count << ",";
    json << "\"progressive_filter_front_unrevealed_point_count\":" << progressive.filter_front_unrevealed_point_count << ",";
    json << "\"progressive_published_face_point_count\":" << progressive.published_face_point_count << ",";
    json << "\"progressive_reveal_source\":\"" << progressive.reveal_source << "\",";
    appendEquipmentJson(json, "cloud_estimate", cloud_equipment);
    json << ",";
    appendEquipmentJson(json, "real_sensors", sensor_reference);
    json << ",";
    appendTargetXyJson(json, target_xy);
    json << "}";
    return json.str();
  }

  static void appendEquipmentJson(std::ostringstream& json,
                                  const std::string& key,
                                  const EquipmentStateSnapshot& state) {
    json << "\"" << key << "\":{";
    json << "\"seen\":" << (state.seen ? "true" : "false") << ",";
    json << "\"source\":\"" << state.source << "\",";
    json << "\"quality\":\"" << state.quality << "\",";
    json << "\"stamp_ns\":" << state.stamp_ns << ",";
    json << "\"attitude\":{";
    json << "\"valid\":" << (state.attitude_valid ? "true" : "false") << ",";
    json << "\"roll_valid\":" << (state.roll_valid ? "true" : "false") << ",";
    json << "\"pitch_valid\":" << (state.pitch_valid ? "true" : "false") << ",";
    json << "\"yaw_valid\":" << (state.yaw_valid ? "true" : "false") << ",";
    json << "\"roll_deg\":" << state.roll_deg << ",";
    json << "\"pitch_deg\":" << state.pitch_deg << ",";
    json << "\"yaw_deg\":" << state.yaw_deg << ",";
    json << "\"ground_plane_rmse\":" << state.ground_plane_rmse << ",";
    json << "\"ground_points\":" << state.ground_points << ",";
    json << "\"wall_points\":" << state.wall_points << ",";
    json << "\"point_count\":" << state.point_count;
    json << "},";
    json << "\"distances\":{";
    json << "\"valid\":" << (state.distances_valid ? "true" : "false") << ",";
    json << "\"left_front_valid\":" << (state.left_front_valid ? "true" : "false") << ",";
    json << "\"left_rear_valid\":" << (state.left_rear_valid ? "true" : "false") << ",";
    json << "\"right_front_valid\":" << (state.right_front_valid ? "true" : "false") << ",";
    json << "\"right_rear_valid\":" << (state.right_rear_valid ? "true" : "false") << ",";
    json << "\"left_front_mm\":" << state.left_front_mm << ",";
    json << "\"left_rear_mm\":" << state.left_rear_mm << ",";
    json << "\"right_front_mm\":" << state.right_front_mm << ",";
    json << "\"right_rear_mm\":" << state.right_rear_mm << ",";
    json << "\"left_front_points\":" << state.left_front_points << ",";
    json << "\"left_rear_points\":" << state.left_rear_points << ",";
    json << "\"right_front_points\":" << state.right_front_points << ",";
    json << "\"right_rear_points\":" << state.right_rear_points;
    json << "}}";
  }

  static void appendTargetXyJson(std::ostringstream& json,
                                 const TargetXySnapshot& state) {
    json << "\"target_xy\":{";
    json << "\"seen\":" << (state.seen ? "true" : "false") << ",";
    json << "\"stamp_ns\":" << state.stamp_ns << ",";
    json << "\"center_x_mm\":" << state.center_x_mm << ",";
    json << "\"center_y_mm\":" << state.center_y_mm << ",";
    json << "\"dx_mm\":" << state.dx_mm << ",";
    json << "\"dy_mm\":" << state.dy_mm << ",";
    json << "\"velocity_x_mm_s\":" << state.velocity_x_mm_s << ",";
    json << "\"velocity_y_mm_s\":" << state.velocity_y_mm_s << ",";
    json << "\"status\":" << state.status << ",";
    json << "\"status_text\":\"" << state.status_text << "\"";
    json << "}";
  }

  void statusTimerCallback(const ros::TimerEvent&) {
    const std::string json = buildStatusJson();
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_status_json_ = json;
    }
    if (status_server_.clientCount() > 0) {
      status_server_.broadcastText(json);
    }
  }

  bool hasLatestCurrentCloud() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return static_cast<bool>(latest_current_cloud_packet_);
  }

  bool hasLatestStableMap() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return static_cast<bool>(latest_stable_map_packet_);
  }

  std::string latest_initialSnapshotReason() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return latest_initial_snapshot_reason_;
  }

  std::uint64_t sessionId() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return session_id_;
  }

  void sendInitialSnapshots(const std::string& reason) {
    std::shared_ptr<const std::vector<std::uint8_t>> current;
    std::shared_ptr<const std::vector<std::uint8_t>> stable;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_initial_snapshot_reason_ = reason;
      current = latest_current_cloud_packet_;
      stable = latest_stable_map_packet_;
    }
    const std::string status = buildStatusJson();
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_status_json_ = status;
    }
    if (!status.empty()) {
      status_server_.broadcastText(status);
    }
    if (stable) {
      cloud_server_.broadcast(stable);
    }
    if (current) {
      cloud_server_.broadcast(current);
    }
  }

  ros::NodeHandle nh_;
  int cloud_port_ = 9001;
  int status_port_ = 9002;
  WebSocketServer cloud_server_;
  WebSocketServer status_server_;
  ros::Subscriber current_sub_;
  ros::Subscriber stable_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber progressive_reveal_sub_;
  ros::Subscriber equipment_state_sub_;
  ros::Subscriber sensor_reference_sub_;
  ros::Subscriber target_xy_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber odom_fallback_sub_;
  ros::Subscriber clock_sub_;
  ros::Timer status_timer_;

  std::string current_cloud_topic_ = "/cloud_registered_body";
  std::string stable_map_topic_ = "/slam/stable_map";
  std::string path_topic_ = "/slam/global_path";
  std::string progressive_reveal_topic_ = "/sim/progressive_reveal_diagnostics";
  std::string equipment_state_topic_ = "/equipment_state";
  std::string sensor_reference_topic_ = "/sensor_reference";
  std::string target_xy_topic_ = "/target_xy";
  std::string odom_topic_ = "/slam/global_odom";
  std::string odom_fallback_topic_ = "/Odometry";
  double current_rate_hz_ = 2.0;
  double current_voxel_size_m_ = 0.15;
  int current_max_points_ = 100000;
  bool transform_current_body_to_map_ = true;
  double stable_rate_hz_ = 0.5;
  double stable_voxel_size_m_ = 0.20;
  int stable_max_points_ = 300000;
  int path_max_points_ = 5000;
  double path_min_step_m_ = 0.05;
  double path_reset_jump_m_ = 5.0;
  double reflector_intensity_threshold_ = 180.0;

  ros::Time last_current_send_;
  ros::Time last_stable_send_;
  ros::Time preferred_odom_seen_until_;
  bool preferred_odom_seen_ = false;

  std::atomic<std::size_t> current_raw_points_{0};
  std::atomic<std::size_t> current_encoded_points_{0};
  std::atomic<std::size_t> stable_raw_points_{0};
  std::atomic<std::size_t> stable_encoded_points_{0};

  mutable std::mutex latest_mutex_;
  std::shared_ptr<const std::vector<std::uint8_t>> latest_current_cloud_packet_;
  std::shared_ptr<const std::vector<std::uint8_t>> latest_stable_map_packet_;
  std::string latest_status_json_;
  std::string latest_initial_snapshot_reason_ = "none";
  std::uint64_t session_id_ = 0;
  ros::Time latest_current_stamp_;
  ros::Time latest_stable_stamp_;

  std::mutex state_mutex_;
  PoseState pose_;
  std::vector<PathPoint> path_;
  std::vector<PathPoint> path_snapshot_;
  ProgressiveRevealState progressive_reveal_;
  EquipmentStateSnapshot cloud_equipment_;
  EquipmentStateSnapshot sensor_reference_;
  TargetXySnapshot target_xy_;
  std::string map_frame_ = "map";
  std::string path_frame_;
  std::string path_snapshot_frame_;
  std::string last_path_reset_reason_ = "none";
  ros::Time last_odom_path_stamp_;
  ros::Time path_snapshot_stamp_;
  ros::Time last_clock_stamp_;
  ros::Time current_clock_stamp_;
  bool path_snapshot_seen_ = false;
  OdomSource odom_source_ = OdomSource::NONE;
  std::uint64_t path_reset_count_ = 0;
};

}  // namespace
}  // namespace mine_slam_web

int main(int argc, char** argv) {
  ros::init(argc, argv, "mine_web_bridge_node");
  try {
    mine_slam_web::MineWebBridgeNode node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL_STREAM("mine_web_bridge_node failed: " << ex.what());
    return 1;
  }
  return 0;
}
