#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Point.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <visualization_msgs/Marker.h>

#include "local_tsdf_mesh/frame_registrar.h"
#include "local_tsdf_mesh/marker_points.h"
#include "local_tsdf_mesh/mesh_cleanup.h"
#include "local_tsdf_mesh/scan_strip_support.h"
#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {
namespace {

using Clock = std::chrono::steady_clock;
using AlignedTsdfPoints =
    std::vector<TsdfPoint, Eigen::aligned_allocator<TsdfPoint>>;

double elapsedMilliseconds(const Clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

std::size_t fieldSize(std::uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

template <typename T>
T readUnaligned(const std::uint8_t* data) {
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

double readNumeric(const std::uint8_t* data,
                   const sensor_msgs::PointField& field) {
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      return readUnaligned<std::int8_t>(data);
    case sensor_msgs::PointField::UINT8:
      return readUnaligned<std::uint8_t>(data);
    case sensor_msgs::PointField::INT16:
      return readUnaligned<std::int16_t>(data);
    case sensor_msgs::PointField::UINT16:
      return readUnaligned<std::uint16_t>(data);
    case sensor_msgs::PointField::INT32:
      return readUnaligned<std::int32_t>(data);
    case sensor_msgs::PointField::UINT32:
      return readUnaligned<std::uint32_t>(data);
    case sensor_msgs::PointField::FLOAT32:
      return readUnaligned<float>(data);
    case sensor_msgs::PointField::FLOAT64:
      return readUnaligned<double>(data);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

Eigen::Vector3f transformPoint(const Eigen::Matrix4f& transform,
                               const Eigen::Vector3f& point) {
  return (transform * Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f))
      .head<3>();
}

std::string formatDouble(double value, int precision = 6) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

template <typename T>
std::string toString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

void addDiagnostic(diagnostic_msgs::DiagnosticStatus& status,
                   const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

struct DecodedFrame {
  pcl::PointCloud<pcl::PointXYZ>::Ptr registration_input{
      new pcl::PointCloud<pcl::PointXYZ>()};
  std::map<std::uint8_t, AlignedTsdfPoints> points_by_sensor;
  std::size_t raw_points = 0;
  std::size_t finite_points = 0;
  std::size_t invalid_points = 0;
  std::size_t registration_range_points = 0;
  std::size_t tsdf_range_points = 0;
  std::size_t surface_range_points = 0;
  std::size_t registration_range_filtered = 0;
  std::size_t tsdf_range_filtered = 0;
  std::size_t surface_range_filtered = 0;
  std::size_t source_topology_points = 0;
  bool has_lidar_id = false;
  bool has_ring = false;
  bool has_azimuth = false;
};

struct TimingMetrics {
  double decode_ms = 0.0;
  double registration_ms = 0.0;
  double integration_ms = 0.0;
  double extraction_ms = 0.0;
  double mesh_cleanup_ms = 0.0;
  double total_ms = 0.0;
};

}  // namespace

class LocalTsdfMeshNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalTsdfMeshNode() : nh_(), pnh_("~") {
    loadConfiguration();
    mesh_cleanup_config_ =
        marchingTetraCleanupConfig(tsdf_config_.voxel_size);
    support_validation_config_ = supportedStripValidationConfig(
        tsdf_config_.voxel_size, support_config_.maximum_mesh_edge_voxels);
    registrar_.reset(new FrameRegistrar(registration_config_));
    volume_.reset(new SparseTsdfVolume(tsdf_config_));

    mesh_pub_ = nh_.advertise<visualization_msgs::Marker>(mesh_topic_, 1, true);
    status_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(status_topic_, 10, true);
    std_msgs::Header startup_header;
    startup_header.stamp = ros::Time::now();
    startup_header.frame_id = expected_input_frame_;
    publishDelete(startup_header);
    cloud_sub_ = nh_.subscribe(input_topic_, 1,
                               &LocalTsdfMeshNode::cloudCallback, this);
    ROS_INFO_STREAM("local_tsdf_mesh: input=" << input_topic_
                    << ", mesh=" << mesh_topic_ << ", status=" << status_topic_
                    << ", sensor_origins=" << sensor_origins_.size()
                    << ", voxel=" << tsdf_config_.voxel_size
                    << " m, window=" << tsdf_config_.window_duration_sec << " s");
  }

 private:
  void loadConfiguration() {
    pnh_.param<std::string>("input_topic", input_topic_, "/points_raw");
    pnh_.param<std::string>("mesh_topic", mesh_topic_,
                            "/local_tsdf_mesh/mesh");
    pnh_.param<std::string>("status_topic", status_topic_,
                            "/local_tsdf_mesh/status");
    pnh_.param<std::string>("expected_input_frame", expected_input_frame_, "");

    pnh_.param("registration/voxel_size", registration_voxel_size_, 0.20);
    pnh_.param("registration/min_range", registration_min_range_, 0.50);
    pnh_.param("registration/max_range", registration_max_range_, 30.0);
    int registration_max_points = 30000;
    int registration_min_points = 200;
    pnh_.param("registration/max_points", registration_max_points, 30000);
    pnh_.param("registration/min_points", registration_min_points, 200);
    registration_max_points_ =
        static_cast<std::size_t>(std::max(1, registration_max_points));
    registration_config_.min_points =
        static_cast<std::size_t>(std::max(5, registration_min_points));
    pnh_.param("registration/maximum_iterations",
               registration_config_.maximum_iterations, 40);
    pnh_.param("registration/correspondence_randomness",
               registration_config_.correspondence_randomness, 10);
    pnh_.param("registration/max_correspondence_distance",
               registration_config_.max_correspondence_distance, 0.60);
    pnh_.param("registration/transformation_epsilon",
               registration_config_.transformation_epsilon, 1e-5);
    pnh_.param("registration/rotation_epsilon",
               registration_config_.rotation_epsilon, 2e-3);
    pnh_.param("registration/fitness_epsilon",
               registration_config_.fitness_epsilon, 1e-5);
    pnh_.param("registration/max_fitness_score",
               registration_config_.max_fitness_score, 0.08);
    pnh_.param("registration/inlier_distance",
               registration_config_.inlier_distance, 0.30);
    pnh_.param("registration/min_inlier_ratio",
               registration_config_.min_inlier_ratio, 0.35);
    pnh_.param("registration/max_translation_m",
               registration_config_.max_translation, 2.0);
    double max_rotation_deg = 15.0;
    pnh_.param("registration/max_rotation_deg", max_rotation_deg, 15.0);
    registration_config_.max_rotation_rad =
        max_rotation_deg * 3.14159265358979323846 / 180.0;
    pnh_.param("registration/allow_unverified_bootstrap",
               registration_config_.allow_unverified_bootstrap, false);
    pnh_.param("registration/allow_near_identity_bootstrap",
               registration_config_.allow_near_identity_bootstrap, false);
    pnh_.param("registration/safe_single_frame_fallback",
               safe_single_frame_fallback_, true);
    pnh_.param("registration/minimum_bootstrap_translation_m",
               registration_config_.minimum_bootstrap_translation, 0.10);
    double minimum_bootstrap_rotation_deg = 0.5;
    pnh_.param("registration/minimum_bootstrap_rotation_deg",
               minimum_bootstrap_rotation_deg, 0.5);
    registration_config_.minimum_bootstrap_rotation_rad =
        minimum_bootstrap_rotation_deg * 3.14159265358979323846 / 180.0;
    pnh_.param("registration/max_prediction_translation_error_m",
               registration_config_.max_prediction_translation_error, 0.75);
    double max_prediction_rotation_error_deg = 10.0;
    pnh_.param("registration/max_prediction_rotation_error_deg",
               max_prediction_rotation_error_deg, 10.0);
    registration_config_.max_prediction_rotation_error_rad =
        max_prediction_rotation_error_deg * 3.14159265358979323846 / 180.0;
    pnh_.param("registration/failures_before_reset", failures_before_reset_, 3);
    failures_before_reset_ = std::max(1, failures_before_reset_);

    pnh_.param("tsdf/integrate_measured_rays",
               tsdf_config_.integrate_measured_rays, true);
    pnh_.param("tsdf/parallel_support_build",
               tsdf_config_.parallel_support_build, true);
    pnh_.param("tsdf/voxel_size", tsdf_config_.voxel_size, 0.10f);
    pnh_.param("tsdf/truncation_distance",
               tsdf_config_.truncation_distance, 0.30f);
    pnh_.param("tsdf/min_range", tsdf_config_.min_range, 0.50f);
    pnh_.param("tsdf/max_range", tsdf_config_.max_range, 30.0f);
    pnh_.param("tsdf/max_weight_per_voxel_per_frame",
               tsdf_config_.max_weight_per_voxel_per_frame, 4.0f);
    pnh_.param("tsdf/window_duration_sec",
               tsdf_config_.window_duration_sec, 1.50);
    int max_window_frames = 15;
    int max_points_per_frame = 12000;
    int max_voxels = 500000;
    pnh_.param("tsdf/max_window_frames", max_window_frames, 15);
    pnh_.param("tsdf/max_points_per_frame", max_points_per_frame, 12000);
    pnh_.param("tsdf/max_voxels", max_voxels, 500000);
    tsdf_config_.max_window_frames =
        static_cast<std::size_t>(std::max(1, max_window_frames));
    tsdf_config_.max_points_per_frame =
        static_cast<std::size_t>(std::max(1, max_points_per_frame));
    tsdf_config_.max_voxels =
        static_cast<std::size_t>(std::max(1000, max_voxels));

    support_config_.voxel_size = tsdf_config_.voxel_size;
    pnh_.param("tsdf/lateral_support/enabled", support_config_.enabled, false);
    pnh_.param("tsdf/lateral_support/min_range", support_config_.min_range,
               8.0f);
    pnh_.param("tsdf/lateral_support/max_range", support_config_.max_range,
               30.0f);
    int support_max_surface_cells = 36000;
    int support_max_candidate_samples = 300000;
    pnh_.param("tsdf/lateral_support/max_surface_cells",
               support_max_surface_cells, 36000);
    pnh_.param("tsdf/lateral_support/max_candidate_samples",
               support_max_candidate_samples, 300000);
    support_config_.max_surface_cells = static_cast<std::size_t>(
        std::max(1, support_max_surface_cells));
    support_config_.max_candidate_samples = static_cast<std::size_t>(
        std::max(1, support_max_candidate_samples));
    pnh_.param("tsdf/lateral_support/surface_spacing_voxels",
               support_config_.surface_spacing_voxels, 0.75f);
    pnh_.param("tsdf/lateral_support/integration_band_voxels",
               support_config_.integration_band_voxels, 1.50f);
    pnh_.param("tsdf/lateral_support/strong_ray_weight",
               support_config_.strong_ray_weight, 0.50f);
    pnh_.param("tsdf/lateral_support/verified_long_ray_weight",
               support_config_.verified_long_ray_weight, 0.25f);
    pnh_.param("tsdf/lateral_support/maximum_weight_per_voxel",
               support_config_.maximum_weight_per_voxel, 1.0f);
    pnh_.param("tsdf/lateral_support/build_surface_rays",
               support_config_.build_surface_rays, true);
    pnh_.param("tsdf/lateral_support/build_indexed_mesh",
               support_config_.build_indexed_mesh, false);
    int support_max_mesh_vertices = 200000;
    int support_max_mesh_triangles = 400000;
    pnh_.param("tsdf/lateral_support/max_mesh_vertices",
               support_max_mesh_vertices, 200000);
    pnh_.param("tsdf/lateral_support/max_mesh_triangles",
               support_max_mesh_triangles, 400000);
    support_config_.max_mesh_vertices = static_cast<std::size_t>(
        std::max(1, support_max_mesh_vertices));
    support_config_.max_mesh_triangles = static_cast<std::size_t>(
        std::max(1, support_max_mesh_triangles));
    pnh_.param("tsdf/lateral_support/maximum_mesh_edge_voxels",
               support_config_.maximum_mesh_edge_voxels, 1.0f);
    pnh_.param("tsdf/lateral_support/mesh_weld_tolerance_voxels",
               support_config_.mesh_weld_tolerance_voxels, 1e-4f);
    pnh_.param("tsdf/lateral_support/minimum_ring_angle_deg",
               support_config_.minimum_ring_angle_deg, 0.20f);
    pnh_.param("tsdf/lateral_support/maximum_ring_angle_deg",
               support_config_.maximum_ring_angle_deg, 5.0f);
    pnh_.param("tsdf/lateral_support/maximum_ring_order_direction_angle_deg",
               support_config_.maximum_ring_order_direction_angle_deg,
               45.0f);
    pnh_.param("tsdf/lateral_support/minimum_match_angle_deg",
               support_config_.minimum_match_angle_deg, 0.12f);
    pnh_.param("tsdf/lateral_support/maximum_match_angle_deg",
               support_config_.maximum_match_angle_deg, 0.30f);
    pnh_.param("tsdf/lateral_support/match_step_factor",
               support_config_.match_step_factor, 1.50f);
    pnh_.param("tsdf/lateral_support/minimum_gap_angle_deg",
               support_config_.minimum_gap_angle_deg, 0.35f);
    pnh_.param("tsdf/lateral_support/maximum_gap_angle_deg",
               support_config_.maximum_gap_angle_deg, 0.65f);
    pnh_.param("tsdf/lateral_support/gap_step_factor",
               support_config_.gap_step_factor, 2.60f);
    pnh_.param("tsdf/lateral_support/maximum_along_log_range_delta",
               support_config_.maximum_along_log_range_delta, 0.05f);
    pnh_.param("tsdf/lateral_support/maximum_along_edge_base_m",
               support_config_.maximum_along_edge_base_m, 0.30f);
    pnh_.param("tsdf/lateral_support/maximum_along_edge_range_ratio",
               support_config_.maximum_along_edge_range_ratio, 0.04f);
    pnh_.param("tsdf/lateral_support/strong_cross_log_range_delta",
               support_config_.strong_cross_log_range_delta, 0.025f);
    pnh_.param("tsdf/lateral_support/strong_cross_edge_base_m",
               support_config_.strong_cross_edge_base_m, 0.60f);
    pnh_.param("tsdf/lateral_support/strong_cross_edge_range_ratio",
               support_config_.strong_cross_edge_range_ratio, 0.055f);
    pnh_.param("tsdf/lateral_support/maximum_cross_log_range_delta",
               support_config_.maximum_cross_log_range_delta, 0.25f);
    pnh_.param("tsdf/lateral_support/maximum_cross_edge_base_m",
               support_config_.maximum_cross_edge_base_m, 1.00f);
    pnh_.param("tsdf/lateral_support/maximum_cross_edge_range_ratio",
               support_config_.maximum_cross_edge_range_ratio, 0.25f);
    pnh_.param("tsdf/lateral_support/maximum_cross_edge_absolute_m",
               support_config_.maximum_cross_edge_absolute_m, 2.50f);
    pnh_.param("tsdf/lateral_support/maximum_quad_normal_angle_deg",
               support_config_.maximum_quad_normal_angle_deg, 25.0f);
    pnh_.param("tsdf/lateral_support/maximum_neighbor_normal_angle_deg",
               support_config_.maximum_neighbor_normal_angle_deg, 15.0f);
    pnh_.param("tsdf/lateral_support/maximum_opposite_edge_length_ratio",
               support_config_.maximum_opposite_edge_length_ratio, 1.80f);
    pnh_.param("tsdf/lateral_support/maximum_planarity_base_m",
               support_config_.maximum_planarity_base_m, 0.15f);
    pnh_.param("tsdf/lateral_support/maximum_planarity_edge_ratio",
               support_config_.maximum_planarity_edge_ratio, 0.08f);
    pnh_.param("tsdf/lateral_support/maximum_neighbor_plane_edge_ratio",
               support_config_.maximum_neighbor_plane_edge_ratio, 0.05f);
    pnh_.param("tsdf/lateral_support/maximum_run_cross_edge_ratio",
               support_config_.maximum_run_cross_edge_ratio, 1.60f);
    int support_minimum_run_quads = 2;
    pnh_.param("tsdf/lateral_support/minimum_run_quads",
               support_minimum_run_quads, 2);
    support_config_.minimum_run_quads =
        static_cast<std::size_t>(std::max(1, support_minimum_run_quads));

    pnh_.param("mesh/minimum_weight", mesh_config_.minimum_weight, 1.0f);
    pnh_.param("mesh/iso_level", mesh_config_.iso_level, 0.0f);
    int max_triangles = 200000;
    pnh_.param("mesh/max_triangles", max_triangles, 200000);
    mesh_config_.max_triangles =
        static_cast<std::size_t>(std::max(1, max_triangles));
    pnh_.param("mesh/parallel_support_validation",
               parallel_support_validation_, false);
    pnh_.param("mesh/publish_rate_hz", mesh_publish_rate_hz_, 0.0);
    pnh_.param("mesh/color_r", mesh_color_r_, 0.92);
    pnh_.param("mesh/color_g", mesh_color_g_, 0.58);
    pnh_.param("mesh/color_b", mesh_color_b_, 0.16);
    pnh_.param("mesh/color_a", mesh_color_a_, 0.72);

    pnh_.param("sensors/require_configured_origin",
               require_configured_sensor_origin_, true);
    pnh_.param("sensors/require_lidar_id", require_lidar_id_, true);
    std::vector<double> flat_origins;
    if (!pnh_.getParam("sensors/origins_xyz", flat_origins)) {
      flat_origins = {0.0, 0.0, 0.0};
    }
    if (flat_origins.empty() || flat_origins.size() % 3U != 0U) {
      throw std::invalid_argument(
          "~sensors/origins_xyz must contain x/y/z triples indexed by lidar_id");
    }
    sensor_origins_.reserve(flat_origins.size() / 3U);
    for (std::size_t index = 0; index < flat_origins.size(); index += 3U) {
      if (!std::isfinite(flat_origins[index]) ||
          !std::isfinite(flat_origins[index + 1U]) ||
          !std::isfinite(flat_origins[index + 2U])) {
        throw std::invalid_argument("~sensors/origins_xyz must be finite");
      }
      const Eigen::Vector3f origin(static_cast<float>(flat_origins[index]),
                                   static_cast<float>(flat_origins[index + 1U]),
                                   static_cast<float>(flat_origins[index + 2U]));
      if (!origin.allFinite()) {
        throw std::invalid_argument(
            "~sensors/origins_xyz exceeds float coordinate range");
      }
      sensor_origins_.push_back(origin);
    }

    const bool registration_parameters_finite =
        std::isfinite(registration_voxel_size_) &&
        std::isfinite(registration_min_range_) &&
        std::isfinite(registration_max_range_) &&
        std::isfinite(registration_config_.max_correspondence_distance) &&
        std::isfinite(registration_config_.transformation_epsilon) &&
        std::isfinite(registration_config_.rotation_epsilon) &&
        std::isfinite(registration_config_.fitness_epsilon) &&
        std::isfinite(registration_config_.max_fitness_score) &&
        std::isfinite(registration_config_.inlier_distance) &&
        std::isfinite(registration_config_.min_inlier_ratio) &&
        std::isfinite(registration_config_.max_translation) &&
        std::isfinite(registration_config_.max_rotation_rad) &&
        std::isfinite(registration_config_.minimum_bootstrap_translation) &&
        std::isfinite(registration_config_.minimum_bootstrap_rotation_rad) &&
        std::isfinite(registration_config_.max_prediction_translation_error) &&
        std::isfinite(registration_config_.max_prediction_rotation_error_rad);
    if (!registration_parameters_finite ||
        !(registration_voxel_size_ > 0.0) ||
        !(registration_min_range_ > 0.0) ||
        !(registration_max_range_ > registration_min_range_)) {
      throw std::invalid_argument("invalid registration preprocessing parameters");
    }
    if (!std::isfinite(mesh_config_.minimum_weight) ||
        !std::isfinite(mesh_config_.iso_level) ||
        !std::isfinite(mesh_publish_rate_hz_) ||
        !std::isfinite(mesh_color_r_) || !std::isfinite(mesh_color_g_) ||
        !std::isfinite(mesh_color_b_) || !std::isfinite(mesh_color_a_) ||
        mesh_config_.minimum_weight <= 0.0f || mesh_config_.iso_level < -1.0f ||
        mesh_config_.iso_level > 1.0f || mesh_publish_rate_hz_ < 0.0) {
      throw std::invalid_argument("invalid mesh parameters");
    }
    if (mesh_publish_rate_hz_ > 0.0) {
      throw std::invalid_argument(
          "mesh/publish_rate_hz must be 0 for current-frame local output");
    }
    const bool volume_surface_enabled =
        tsdf_config_.integrate_measured_rays ||
        (support_config_.enabled && support_config_.build_surface_rays);
    const bool direct_surface_enabled =
        support_config_.enabled && support_config_.build_indexed_mesh;
    if (!volume_surface_enabled && !direct_surface_enabled) {
      throw std::invalid_argument(
          "no active surface producer: enable measured TSDF, support rays, "
          "or indexed scan strips");
    }
  }

  bool decodeCloud(const sensor_msgs::PointCloud2& message, DecodedFrame& decoded,
                   std::string& reason) const {
    if (message.width == 0 || message.height == 0 ||
        static_cast<std::size_t>(message.height) >
            std::numeric_limits<std::size_t>::max() /
                static_cast<std::size_t>(message.width)) {
      reason = "invalid_cloud_dimensions";
      return false;
    }
    decoded.raw_points =
        static_cast<std::size_t>(message.width) * message.height;
    if (message.is_bigendian) {
      reason = "big_endian_cloud_unsupported";
      return false;
    }
    if (message.point_step == 0 || message.row_step == 0) {
      reason = "invalid_point_layout";
      return false;
    }
    if (static_cast<std::size_t>(message.width) >
        std::numeric_limits<std::size_t>::max() /
            static_cast<std::size_t>(message.point_step)) {
      reason = "point_layout_overflow";
      return false;
    }
    const std::size_t minimum_row_bytes =
        static_cast<std::size_t>(message.width) * message.point_step;
    if (message.row_step < minimum_row_bytes) {
      reason = "invalid_row_step";
      return false;
    }
    const std::size_t preceding_rows =
        static_cast<std::size_t>(message.height - 1U);
    if (preceding_rows >
        (std::numeric_limits<std::size_t>::max() - minimum_row_bytes) /
            static_cast<std::size_t>(message.row_step)) {
      reason = "point_layout_overflow";
      return false;
    }
    const std::size_t required_data_bytes =
        preceding_rows * message.row_step + minimum_row_bytes;
    if (required_data_bytes > message.data.size()) {
      reason = "truncated_point_data";
      return false;
    }
    const auto* x_field = findField(message, "x");
    const auto* y_field = findField(message, "y");
    const auto* z_field = findField(message, "z");
    const auto* lidar_field = findField(message, "lidar_id");
    const auto* ring_field = findField(message, "ring");
    const auto* azimuth_field = findField(message, "azimuth");
    if (!x_field || !y_field || !z_field) {
      reason = "missing_xyz_fields";
      return false;
    }
    if (!lidar_field && require_lidar_id_) {
      reason = "missing_lidar_id_field";
      return false;
    }
    const std::array<const sensor_msgs::PointField*, 3> xyz =
        {{x_field, y_field, z_field}};
    for (const auto* field : xyz) {
      const std::size_t size = fieldSize(field->datatype);
      if (size == 0 || field->count == 0 ||
          static_cast<std::size_t>(field->offset) + size > message.point_step) {
        reason = "unsupported_xyz_layout";
        return false;
      }
    }
    if (lidar_field) {
      const std::size_t size = fieldSize(lidar_field->datatype);
      if (size == 0 || lidar_field->count == 0 ||
          static_cast<std::size_t>(lidar_field->offset) + size >
              message.point_step) {
        reason = "unsupported_lidar_id_layout";
        return false;
      }
      decoded.has_lidar_id = true;
    }
    const auto optionalFieldUsable = [&message](
                                         const sensor_msgs::PointField* field) {
      if (!field) {
        return false;
      }
      const std::size_t size = fieldSize(field->datatype);
      return size > 0 && field->count > 0 &&
             static_cast<std::size_t>(field->offset) + size <=
                 message.point_step;
    };
    decoded.has_ring = optionalFieldUsable(ring_field);
    decoded.has_azimuth = optionalFieldUsable(azimuth_field);

    const bool build_registration_cloud =
        !(safe_single_frame_fallback_ &&
          !registration_config_.allow_unverified_bootstrap);
    if (build_registration_cloud) {
      decoded.registration_input->points.reserve(decoded.raw_points);
    }
    for (std::uint32_t row = 0; row < message.height; ++row) {
      const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
      for (std::uint32_t column = 0; column < message.width; ++column) {
        const std::size_t offset =
            row_offset + static_cast<std::size_t>(column) * message.point_step;
        if (offset + message.point_step > message.data.size()) {
          reason = "truncated_point_data";
          return false;
        }
        const auto* point_data = message.data.data() + offset;
        const double x = readNumeric(point_data + x_field->offset, *x_field);
        const double y = readNumeric(point_data + y_field->offset, *y_field);
        const double z = readNumeric(point_data + z_field->offset, *z_field);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          ++decoded.invalid_points;
          continue;
        }
        const Eigen::Vector3f point(static_cast<float>(x), static_cast<float>(y),
                                    static_cast<float>(z));
        if (!point.allFinite()) {
          ++decoded.invalid_points;
          continue;
        }
        std::uint8_t lidar_id = 0;
        if (lidar_field) {
          const double raw_id =
              readNumeric(point_data + lidar_field->offset, *lidar_field);
          if (!std::isfinite(raw_id) || raw_id < 0.0 || raw_id > 255.0 ||
              std::floor(raw_id) != raw_id) {
            ++decoded.invalid_points;
            continue;
          }
          lidar_id = static_cast<std::uint8_t>(raw_id);
        }
        if (require_configured_sensor_origin_ &&
            static_cast<std::size_t>(lidar_id) >= sensor_origins_.size()) {
          reason = "missing_sensor_origin_" +
                   toString(static_cast<unsigned int>(lidar_id));
          return false;
        }
        ++decoded.finite_points;

        const Eigen::Vector3f sensor_origin =
            static_cast<std::size_t>(lidar_id) < sensor_origins_.size()
                ? sensor_origins_[lidar_id]
                : Eigen::Vector3f::Zero();
        const float sensor_range = (point - sensor_origin).norm();
        const bool in_tsdf_range =
            std::isfinite(sensor_range) &&
            sensor_range >= tsdf_config_.min_range &&
            sensor_range <= tsdf_config_.max_range;
        const bool in_support_range =
            support_config_.enabled && std::isfinite(sensor_range) &&
            sensor_range >= support_config_.min_range &&
            sensor_range <= support_config_.max_range;
        const bool keep_for_tsdf =
            tsdf_config_.integrate_measured_rays && in_tsdf_range;
        const bool keep_for_support =
            support_config_.enabled &&
            (support_config_.build_surface_rays ||
             support_config_.build_indexed_mesh) &&
            in_support_range;
        if (in_tsdf_range) {
          ++decoded.tsdf_range_points;
        } else {
          ++decoded.tsdf_range_filtered;
        }
        // Keep the union of the measured-TSDF and direct-support ranges in
        // the decoded sensor clouds. SparseTsdfVolume independently applies
        // the TSDF range when selecting measured rays, while the strip
        // builder applies its own range. This permits a cheap near-field TSDF
        // and a farther direct strip without discarding either input first.
        if (keep_for_tsdf || keep_for_support) {
          TsdfPoint sample;
          sample.position = point;
          double raw_ring = std::numeric_limits<double>::quiet_NaN();
          double raw_azimuth = std::numeric_limits<double>::quiet_NaN();
          if (decoded.has_ring) {
            raw_ring =
                readNumeric(point_data + ring_field->offset, *ring_field);
          }
          if (decoded.has_azimuth) {
            raw_azimuth =
                readNumeric(point_data + azimuth_field->offset, *azimuth_field);
          }
          if (assignSourceTopologyFields(decoded.has_ring, raw_ring,
                                         decoded.has_azimuth, raw_azimuth,
                                         sample)) {
            ++decoded.source_topology_points;
          }
          if (!std::isfinite(sample.azimuth)) {
            const Eigen::Vector3f sensor_ray = point - sensor_origin;
            sample.azimuth =
                std::atan2(sensor_ray.y(), sensor_ray.x());
          }
          decoded.points_by_sensor[lidar_id].push_back(sample);
          ++decoded.surface_range_points;
        } else {
          ++decoded.surface_range_filtered;
        }

        // Registration operates on the fused cloud geometry, independently
        // from the per-sensor ray range used by TSDF integration.
        const float registration_range = point.norm();
        if (std::isfinite(registration_range) &&
            registration_range >= registration_min_range_ &&
            registration_range <= registration_max_range_) {
          if (build_registration_cloud) {
            decoded.registration_input->points.emplace_back(
                point.x(), point.y(), point.z());
          }
          ++decoded.registration_range_points;
        } else {
          ++decoded.registration_range_filtered;
        }
      }
    }
    decoded.registration_input->width =
        static_cast<std::uint32_t>(decoded.registration_input->points.size());
    decoded.registration_input->height = 1;
    decoded.registration_input->is_dense = true;
    if (decoded.surface_range_points == 0) {
      reason = "no_surface_range_points";
      return false;
    }
    reason = "accepted";
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr prepareRegistrationCloud(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (!input || input->empty()) {
      return filtered;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    const float leaf = static_cast<float>(registration_voxel_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    if (filtered->size() <= registration_max_points_) {
      return filtered;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr limited(
        new pcl::PointCloud<pcl::PointXYZ>());
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
        static_cast<double>(filtered->size()) /
        static_cast<double>(registration_max_points_)));
    limited->points.reserve(registration_max_points_);
    for (std::size_t index = 0; index < filtered->size(); index += stride) {
      limited->points.push_back(filtered->points[index]);
    }
    limited->width = static_cast<std::uint32_t>(limited->points.size());
    limited->height = 1;
    limited->is_dense = true;
    return limited;
  }

  bool makeSensorClouds(
      const DecodedFrame& decoded, const Eigen::Matrix4f& reference_from_cloud,
      std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>>& sensors,
      std::string& reason) const {
    sensors.clear();
    sensors.reserve(decoded.points_by_sensor.size());
    for (const auto& entry : decoded.points_by_sensor) {
      const std::size_t sensor_id = entry.first;
      if (sensor_id >= sensor_origins_.size() &&
          require_configured_sensor_origin_) {
        reason = "missing_sensor_origin_" + toString(sensor_id);
        return false;
      }
      const Eigen::Vector3f local_origin =
          sensor_id < sensor_origins_.size() ? sensor_origins_[sensor_id]
                                             : Eigen::Vector3f::Zero();
      SensorCloud sensor;
      sensor.sensor_id = entry.first;
      sensor.origin = transformPoint(reference_from_cloud, local_origin);
      sensor.points.reserve(entry.second.size());
      for (const auto& sample : entry.second) {
        TsdfPoint transformed = sample;
        transformed.position =
            transformPoint(reference_from_cloud, sample.position);
        sensor.points.push_back(transformed);
      }
      sensors.push_back(std::move(sensor));
    }
    reason = "accepted";
    return !sensors.empty();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const Clock::time_point total_start = Clock::now();
    TimingMetrics timing;
    RegistrationResult registration;
    IntegrationResult integration;
    std::size_t mesh_triangles = last_mesh_triangles_;
    bool mesh_limit = last_mesh_limit_reached_;

    if (!message) {
      ++rejected_frames_;
      ROS_ERROR_THROTTLE(1.0, "local_tsdf_mesh: received a null cloud pointer");
      return;
    }
    if (message->header.stamp.isZero() || message->header.frame_id.empty()) {
      ++rejected_frames_;
      if (initialized_) {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
      }
      timing.total_ms = elapsedMilliseconds(total_start);
      std_msgsHeaderFallback(*message, "INPUT_REJECTED", "invalid_header",
                             registration, integration, timing, 0, mesh_triangles,
                             mesh_limit);
      return;
    }
    if (!expected_input_frame_.empty() &&
        message->header.frame_id != expected_input_frame_) {
      ++rejected_frames_;
      if (initialized_) {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
      }
      timing.total_ms = elapsedMilliseconds(total_start);
      std_msgsHeaderFallback(*message, "INPUT_REJECTED",
                             "unexpected_input_frame", registration,
                             integration, timing, 0, mesh_triangles, mesh_limit);
      return;
    }

    const Clock::time_point decode_start = Clock::now();
    DecodedFrame decoded;
    std::string decode_reason;
    if (!decodeCloud(*message, decoded, decode_reason)) {
      timing.decode_ms = elapsedMilliseconds(decode_start);
      timing.total_ms = elapsedMilliseconds(total_start);
      ++rejected_frames_;
      if (initialized_) {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
      }
      publishStatus(message->header, "INPUT_REJECTED", decode_reason, decoded,
                    registration, integration, timing, 0, mesh_triangles,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    const bool safe_single_frame_mode =
        safe_single_frame_fallback_ &&
        !registration_config_.allow_unverified_bootstrap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (!safe_single_frame_mode) {
      registration_cloud = prepareRegistrationCloud(decoded.registration_input);
    }
    timing.decode_ms = elapsedMilliseconds(decode_start);
    if (!safe_single_frame_mode &&
        registration_cloud->size() < registration_config_.min_points) {
      timing.total_ms = elapsedMilliseconds(total_start);
      ++rejected_frames_;
      if (initialized_) {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
      }
      publishStatus(message->header, "INPUT_REJECTED",
                    "too_few_registration_points", decoded, registration,
                    integration, timing, registration_cloud->size(),
                    mesh_triangles, mesh_limit,
                    diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }

    const bool stamp_regressed =
        have_received_stamp_ && message->header.stamp <= last_received_stamp_;
    const bool frame_changed =
        initialized_ && message->header.frame_id != input_frame_id_;
    if (!initialized_ || stamp_regressed || frame_changed) {
      const std::string reason = !initialized_      ? "initial_bootstrap"
                                 : frame_changed    ? "input_frame_changed"
                                                    : "stamp_regression";
      if (initialized_) {
        publishDelete(message->header);
        ++reset_count_;
      }
      resetMappingState();
      bootstrap(*message, decoded, registration_cloud, reason, timing,
                total_start);
      return;
    }
    last_received_stamp_ = message->header.stamp;
    have_received_stamp_ = true;

    // A repetitive tunnel does not provide enough longitudinal structure to
    // establish a trustworthy pure-ICP motion prior. In the safe configuration
    // do useful current-frame TSDF reconstruction without running an expensive
    // registration that is guaranteed to be rejected. This mode never mixes
    // points from different timestamps.
    if (safe_single_frame_mode && !motion_prediction_valid_) {
      rebuildSingleFrame(*message, decoded, registration_cloud, timing,
                         total_start);
      return;
    }

    const Clock::time_point registration_start = Clock::now();
    registration = registrar_->align(registration_cloud, previous_cloud_,
                                     last_relative_transform_);
    timing.registration_ms = elapsedMilliseconds(registration_start);
    if (registration.accepted) {
      registration.reason = motionConsistencyRejectReason(
          registration.target_from_source, last_relative_transform_,
          motion_prediction_valid_, registration_config_,
          &last_prediction_translation_error_,
          &last_prediction_rotation_error_rad_);
      registration.accepted = registration.reason == "accepted";
    }
    if (!registration.accepted) {
      ++consecutive_registration_failures_;
      ++registration_failures_total_;
      ++rejected_frames_;
      if (consecutive_registration_failures_ >= failures_before_reset_) {
        publishDelete(message->header);
        ++reset_count_;
        resetMappingState();
        bootstrap(*message, decoded, registration_cloud,
                  "registration_failures_reset:" + registration.reason, timing,
                  total_start);
      } else {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
        timing.total_ms = elapsedMilliseconds(total_start);
        publishStatus(message->header, "FALLBACK_CLEAR", registration.reason,
                      decoded, registration, integration, timing,
                      registration_cloud->size(), mesh_triangles, mesh_limit,
                      diagnostic_msgs::DiagnosticStatus::WARN);
      }
      return;
    }

    const Eigen::Matrix4f candidate_reference_from_cloud =
        reference_from_current_ * registration.target_from_source;
    if (!candidate_reference_from_cloud.allFinite()) {
      registration.accepted = false;
      registration.reason = "nonfinite_accumulated_pose";
      ++consecutive_registration_failures_;
      ++registration_failures_total_;
      ++rejected_frames_;
      if (consecutive_registration_failures_ >= failures_before_reset_) {
        publishDelete(message->header);
        ++reset_count_;
        resetMappingState();
        bootstrap(*message, decoded, registration_cloud,
                  "registration_failures_reset:" + registration.reason,
                  timing, total_start);
      } else {
        publishDelete(message->header);
        mesh_triangles = 0;
        mesh_limit = false;
        timing.total_ms = elapsedMilliseconds(total_start);
        publishStatus(message->header, "FALLBACK_CLEAR", registration.reason,
                      decoded, registration, integration, timing,
                      registration_cloud->size(), mesh_triangles, mesh_limit,
                      diagnostic_msgs::DiagnosticStatus::ERROR);
      }
      return;
    }

    // Registration remains useful even if integration is rejected. Advancing the
    // adjacent-frame target avoids making the next ICP bridge an ever-growing gap.
    reference_from_current_ = candidate_reference_from_cloud;
    last_relative_transform_ = registration.target_from_source;
    motion_prediction_valid_ = true;
    previous_cloud_ = registration_cloud;
    consecutive_registration_failures_ = 0;

    std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>> sensors;
    std::string sensor_reason;
    if (!makeSensorClouds(decoded, reference_from_current_, sensors,
                          sensor_reason)) {
      ++rejected_frames_;
      publishDelete(message->header);
      mesh_triangles = 0;
      mesh_limit = false;
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message->header, "FALLBACK_CLEAR", sensor_reason, decoded,
                    registration, integration, timing, registration_cloud->size(),
                    mesh_triangles, mesh_limit,
                    diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }

    const Clock::time_point integration_start = Clock::now();
    integration = volume_->integrateFrame(
        sensors, message->header.stamp.toSec(), support_config_);
    timing.integration_ms = elapsedMilliseconds(integration_start);
    if (!integration.accepted) {
      ++rejected_frames_;
      rejectSurface(integration, integration.reason);
      publishDelete(message->header);
      mesh_triangles = 0;
      mesh_limit = false;
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message->header, "FALLBACK_CLEAR", integration.reason, decoded,
                    registration, integration, timing, registration_cloud->size(),
                    mesh_triangles, mesh_limit,
                    diagnostic_msgs::DiagnosticStatus::WARN);
      return;
    }

    if (integration.volume_updated) {
      // Tracking integrates directly into the active volume, so the volume
      // commit is independent from the later direct-layer/output validation.
      commitVolumeResult(integration, message->header);
    }
    IndexedMesh mesh;
    if (integration.volume_updated) {
      const Clock::time_point extraction_start = Clock::now();
      mesh = volume_->extractMesh(
          mesh_config_, reference_from_current_.block<3, 1>(0, 3));
      timing.extraction_ms = elapsedMilliseconds(extraction_start);
      mesh = prepareMeshForPublish(mesh, integration, mesh_limit,
                                   timing.mesh_cleanup_ms);
    } else {
      prepareStatelessSurface(integration, mesh_limit, timing, mesh);
    }
    if (!integration.surface_prepared) {
      ++rejected_frames_;
      rejectSurface(integration, integration.surface_output_reason);
      publishDelete(message->header, true);
      mesh_triangles = 0U;
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message->header, "SURFACE_REJECTED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(),
                    mesh_triangles, mesh_limit,
                    diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    mesh_triangles = publishMesh(mesh, message->header, reference_from_current_);
    if (mesh_triangles == 0U) {
      ++rejected_frames_;
      rejectSurface(integration, "publish_empty");
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message->header, "SURFACE_REJECTED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(), 0U,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    acceptPublishedSurface(integration);
    last_mesh_triangles_ = mesh_triangles;
    last_mesh_limit_reached_ = mesh_limit;
    timing.total_ms = elapsedMilliseconds(total_start);
    publishStatus(message->header, "TRACKING", "accepted", decoded, registration,
                  integration, timing, registration_cloud->size(), mesh_triangles,
                  mesh_limit, diagnostic_msgs::DiagnosticStatus::OK);
  }

  void bootstrap(const sensor_msgs::PointCloud2& message,
                 const DecodedFrame& decoded,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& registration_cloud,
                 const std::string& reset_reason, TimingMetrics timing,
                 const Clock::time_point& total_start) {
    RegistrationResult registration;
    registration.accepted = true;
    registration.reason = "bootstrap_identity";
    registration.quality.converged = true;
    registration.quality.transform_finite = true;
    registration.quality.inlier_ratio = 1.0;
    registration.quality.source_points = registration_cloud->size();
    registration.quality.target_points = registration_cloud->size();

    std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>> sensors;
    std::string sensor_reason;
    IntegrationResult integration;
    if (!makeSensorClouds(decoded, Eigen::Matrix4f::Identity(), sensors,
                          sensor_reason)) {
      ++rejected_frames_;
      clearCurrentLocalVolume();
      publishDelete(message.header);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "RESET_FAILED", sensor_reason, decoded,
                    registration, integration, timing, registration_cloud->size(),
                    0, false, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    std::unique_ptr<SparseTsdfVolume> replacement(
        new SparseTsdfVolume(tsdf_config_));
    const Clock::time_point integration_start = Clock::now();
    integration = replacement->integrateFrame(
        sensors, message.header.stamp.toSec(), support_config_);
    timing.integration_ms = elapsedMilliseconds(integration_start);
    if (!integration.accepted) {
      ++rejected_frames_;
      rejectSurface(integration, integration.reason);
      clearCurrentLocalVolume();
      publishDelete(message.header);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "RESET_FAILED", integration.reason, decoded,
                    registration, integration, timing, registration_cloud->size(),
                    0, false, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    bool mesh_limit = false;
    IndexedMesh mesh;
    if (integration.volume_updated) {
      const Clock::time_point extraction_start = Clock::now();
      mesh = replacement->extractMesh(mesh_config_);
      timing.extraction_ms = elapsedMilliseconds(extraction_start);
      mesh = prepareMeshForPublish(mesh, integration, mesh_limit,
                                   timing.mesh_cleanup_ms);
    } else {
      prepareStatelessSurface(integration, mesh_limit, timing, mesh);
    }
    if (!integration.surface_prepared) {
      ++rejected_frames_;
      rejectSurface(integration, integration.surface_output_reason);
      publishDelete(message.header, true);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "RESET_FAILED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(), 0,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    const std::size_t mesh_triangles =
        publishMesh(mesh, message.header, Eigen::Matrix4f::Identity());
    if (mesh_triangles == 0U) {
      ++rejected_frames_;
      rejectSurface(integration, "publish_empty");
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "RESET_FAILED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(), 0,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }

    volume_.swap(replacement);
    initialized_ = true;
    input_frame_id_ = message.header.frame_id;
    previous_cloud_ = registration_cloud;
    reference_from_current_.setIdentity();
    last_relative_transform_.setIdentity();
    last_received_stamp_ = message.header.stamp;
    have_received_stamp_ = true;
    consecutive_registration_failures_ = 0;
    if (integration.volume_updated) {
      commitVolumeResult(integration, message.header);
    } else {
      latest_integrated_header_ = std_msgs::Header();
    }
    acceptPublishedSurface(integration);
    last_mesh_triangles_ = mesh_triangles;
    last_mesh_limit_reached_ = mesh_limit;
    timing.total_ms = elapsedMilliseconds(total_start);
    publishStatus(message.header,
                  reset_reason == "initial_bootstrap" ? "BOOTSTRAP"
                                                       : "RESET_BOOTSTRAP",
                  reset_reason, decoded, registration, integration, timing,
                  registration_cloud->size(), mesh_triangles,
                  mesh_limit,
                  diagnostic_msgs::DiagnosticStatus::WARN);
  }

  void rebuildSingleFrame(
      const sensor_msgs::PointCloud2& message, const DecodedFrame& decoded,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& registration_cloud,
      TimingMetrics timing, const Clock::time_point& total_start) {
    RegistrationResult registration;
    registration.reason = "not_run_unverified_motion";
    registration.quality.source_points = registration_cloud->size();
    registration.quality.target_points = previous_cloud_ ? previous_cloud_->size() : 0;

    std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>> sensors;
    std::string sensor_reason;
    IntegrationResult integration;
    if (!makeSensorClouds(decoded, Eigen::Matrix4f::Identity(), sensors,
                          sensor_reason)) {
      ++rejected_frames_;
      clearCurrentLocalVolume();
      publishDelete(message.header);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "SINGLE_FRAME_REJECTED", sensor_reason,
                    decoded, registration, integration, timing,
                    registration_cloud->size(), 0, false,
                    diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }

    // Build the replacement in a temporary volume so a malformed frame cannot
    // corrupt the last accepted volume. Rejection publishes DELETE because the
    // previous body-local mesh is not aligned to the new current frame.
    std::unique_ptr<SparseTsdfVolume> replacement(
        new SparseTsdfVolume(tsdf_config_));
    const Clock::time_point integration_start = Clock::now();
    integration = replacement->integrateFrame(
        sensors, message.header.stamp.toSec(), support_config_);
    timing.integration_ms = elapsedMilliseconds(integration_start);
    if (!integration.accepted) {
      ++rejected_frames_;
      rejectSurface(integration, integration.reason);
      clearCurrentLocalVolume();
      publishDelete(message.header);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "SINGLE_FRAME_REJECTED", integration.reason,
                    decoded, registration, integration, timing,
                    registration_cloud->size(), 0, false,
                    diagnostic_msgs::DiagnosticStatus::WARN);
      return;
    }
    bool mesh_limit = false;
    IndexedMesh mesh;
    if (integration.volume_updated) {
      const Clock::time_point extraction_start = Clock::now();
      mesh = replacement->extractMesh(mesh_config_);
      timing.extraction_ms = elapsedMilliseconds(extraction_start);
      mesh = prepareMeshForPublish(mesh, integration, mesh_limit,
                                   timing.mesh_cleanup_ms);
    } else {
      prepareStatelessSurface(integration, mesh_limit, timing, mesh);
    }
    if (!integration.surface_prepared) {
      ++rejected_frames_;
      rejectSurface(integration, integration.surface_output_reason);
      clearCurrentLocalVolume();
      publishDelete(message.header, true);
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "SINGLE_FRAME_REJECTED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(), 0,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }
    const std::size_t mesh_triangles =
        publishMesh(mesh, message.header, Eigen::Matrix4f::Identity());
    if (mesh_triangles == 0U) {
      ++rejected_frames_;
      rejectSurface(integration, "publish_empty");
      clearCurrentLocalVolume();
      timing.total_ms = elapsedMilliseconds(total_start);
      publishStatus(message.header, "SINGLE_FRAME_REJECTED",
                    integration.surface_output_reason, decoded, registration,
                    integration, timing, registration_cloud->size(), 0,
                    mesh_limit, diagnostic_msgs::DiagnosticStatus::ERROR);
      return;
    }

    volume_.swap(replacement);
    previous_cloud_ = registration_cloud;
    reference_from_current_.setIdentity();
    last_relative_transform_.setIdentity();
    motion_prediction_valid_ = false;
    consecutive_registration_failures_ = 0;
    input_frame_id_ = message.header.frame_id;
    if (integration.volume_updated) {
      commitVolumeResult(integration, message.header);
    } else {
      latest_integrated_header_ = std_msgs::Header();
    }
    acceptPublishedSurface(integration);
    last_mesh_triangles_ = mesh_triangles;
    last_mesh_limit_reached_ = mesh_limit;
    timing.total_ms = elapsedMilliseconds(total_start);
    publishStatus(message.header, "SINGLE_FRAME_FALLBACK",
                  "unverified_motion", decoded, registration, integration,
                  timing, registration_cloud->size(), mesh_triangles, mesh_limit,
                  diagnostic_msgs::DiagnosticStatus::WARN);
  }

  bool prepareStatelessSurface(IntegrationResult& integration,
                               bool& mesh_limit, TimingMetrics& timing,
                               IndexedMesh& prepared_mesh) {
    if (integration.volume_updated) {
      return true;
    }
    const IndexedMesh empty_base;
    prepared_mesh = prepareMeshForPublish(
        empty_base, integration, mesh_limit, timing.mesh_cleanup_ms);
    return integration.surface_prepared;
  }

  IndexedMesh prepareMeshForPublish(const IndexedMesh& raw_base,
                                    IntegrationResult& integration,
                                    bool& mesh_limit, double& cleanup_ms) {
    const Clock::time_point cleanup_start = Clock::now();
    const bool support_requested =
        support_config_.enabled && support_config_.build_indexed_mesh;
    const IndexedMesh empty_support;
    const IndexedMesh& raw_support =
        support_requested ? integration.support_mesh : empty_support;
    LayeredMeshPreparationResult prepared = prepareLayeredMeshForPublish(
        raw_base, raw_support, mesh_config_.max_triangles,
        mesh_cleanup_config_, support_validation_config_,
        kOutputTopologyEquivalenceToleranceM,
        parallel_support_validation_);
    cleanup_ms = elapsedMilliseconds(cleanup_start);

    last_mesh_cleanup_stats_ = prepared.base_cleanup;
    last_support_validation_stats_ = prepared.support_validation;
    last_mesh_preparation_timings_ = prepared.timings;
    last_support_validation_parallel_ =
        prepared.support_validation_parallel;
    last_support_validation_launch_failed_ =
        prepared.support_validation_launch_failed;
    last_base_output_topology_ = prepared.base_output_topology;
    last_support_output_topology_ = prepared.support_output_topology;
    last_combined_topology_ = prepared.combined_topology;
    integration.support_mesh_applied = prepared.support_applied;
    integration.support_mesh_budget_limited =
        integration.support_mesh_budget_limited ||
        prepared.support_budget_limited;
    if (!support_config_.enabled) {
      integration.support_mesh_apply_reason = "support_disabled";
    } else if (!support_config_.build_indexed_mesh) {
      integration.support_mesh_apply_reason = "indexed_mesh_disabled";
    } else {
      integration.support_mesh_apply_reason = prepared.support_reason;
    }
    const bool base_applied = prepared.base_cleanup.output_triangles > 0U;
    const bool support_expected =
        integration.support_mesh_accepted || !raw_support.triangles.empty();
    integration.surface_prepared = false;
    integration.surface_output_accepted = false;
    integration.mesh_published_this_frame = false;
    if (base_applied && prepared.support_applied) {
      integration.surface_mode = "hybrid";
    } else if (prepared.support_applied) {
      integration.surface_mode = "direct_only";
    } else if (base_applied && support_requested && support_expected) {
      integration.surface_mode = "tsdf_only_degraded";
    } else if (base_applied) {
      integration.surface_mode = "tsdf_only";
    } else {
      integration.surface_mode = "none";
    }
    if (!base_applied && !prepared.support_applied) {
      integration.surface_output_reason =
          integration.volume_updated && !support_expected
              ? "empty_base_mesh"
              : prepared.support_reason;
    } else if (prepared.mesh.triangles.empty()) {
      integration.surface_output_reason = "empty_mesh";
    } else {
      integration.surface_prepared = true;
      integration.surface_output_reason = "prepared";
    }
    mesh_limit = raw_base.triangle_limit_reached ||
                 prepared.mesh.triangle_limit_reached ||
                 integration.support_mesh_budget_limited;
    return std::move(prepared.mesh);
  }

  void commitVolumeResult(IntegrationResult& integration,
                          const std_msgs::Header& header) {
    if (!integration.volume_updated || integration.volume_committed) {
      return;
    }
    integration.volume_committed = true;
    ++integrated_frames_;
    latest_integrated_header_ = header;
  }

  void acceptPublishedSurface(IntegrationResult& integration) {
    integration.surface_prepared = true;
    integration.surface_output_accepted = true;
    integration.mesh_published_this_frame = true;
    integration.surface_output_reason = "published";
    ++surface_frames_;
    if (integration.support_mesh_applied) {
      ++direct_applied_frames_;
    }
  }

  void rejectSurface(IntegrationResult& integration,
                     const std::string& reason) {
    integration.surface_output_accepted = false;
    integration.mesh_published_this_frame = false;
    if (!reason.empty()) {
      integration.surface_output_reason = reason;
    }
    ++surface_rejected_frames_;
  }

  void clearCurrentLocalVolume() {
    volume_->clear();
    latest_integrated_header_ = std_msgs::Header();
  }

  std::size_t publishMesh(const IndexedMesh& mesh,
                          const std_msgs::Header& latest_header,
                          const Eigen::Matrix4f& reference_from_current) {
    if (mesh.triangles.empty()) {
      publishDelete(latest_header, true);
      return 0;
    }
    const Eigen::Matrix4f current_from_reference = reference_from_current.inverse();
    if (!current_from_reference.allFinite()) {
      ROS_ERROR_THROTTLE(1.0,
                         "local_tsdf_mesh: cannot invert accumulated local pose");
      publishDelete(latest_header, true);
      return 0;
    }
    visualization_msgs::Marker marker;
    marker.header = latest_header;
    marker.ns = "local_tsdf_mesh";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = static_cast<float>(std::max(0.0, std::min(1.0, mesh_color_r_)));
    marker.color.g = static_cast<float>(std::max(0.0, std::min(1.0, mesh_color_g_)));
    marker.color.b = static_cast<float>(std::max(0.0, std::min(1.0, mesh_color_b_)));
    marker.color.a = static_cast<float>(std::max(0.01, std::min(1.0, mesh_color_a_)));
    TriangleListPointResult marker_points =
        buildTriangleListPoints(mesh, current_from_reference);
    if (marker_points.rejected_triangles > 0U) {
      ROS_ERROR_THROTTLE(1.0, "local_tsdf_mesh: invalid mesh triangle");
      publishDelete(latest_header, true);
      return 0;
    }
    marker.points.swap(marker_points.points);
    if (marker.points.empty() || marker.points.size() % 3U != 0U ||
        marker.points.size() / 3U != mesh.triangles.size()) {
      ROS_ERROR("local_tsdf_mesh: internal triangle index validation failed");
      publishDelete(latest_header, true);
      return 0;
    }
    mesh_pub_.publish(marker);
    last_mesh_event_stamp_ = latest_header.stamp;
    last_mesh_publish_stamp_ = latest_header.stamp;
    return marker.points.size() / 3U;
  }

  void publishDelete(const std_msgs::Header& header,
                     bool preserve_cleanup_stats = false) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "local_tsdf_mesh";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    mesh_pub_.publish(marker);
    last_mesh_triangles_ = 0;
    last_mesh_limit_reached_ = false;
    last_mesh_event_stamp_ = header.stamp;
    // A clear must not consume the rate-limit slot for the next valid mesh.
    last_mesh_publish_stamp_ = ros::Time();
    if (!preserve_cleanup_stats) {
      last_mesh_cleanup_stats_ = MeshCleanupStats();
      last_support_validation_stats_ = MeshCleanupStats();
      last_mesh_preparation_timings_ =
          LayeredMeshPreparationResult::Timings();
      last_support_validation_parallel_ = false;
      last_support_validation_launch_failed_ = false;
      last_base_output_topology_ = MeshTopologyValidationResult();
      last_support_output_topology_ = MeshTopologyValidationResult();
      last_combined_topology_ = MeshTopologyValidationResult();
    }
  }

  void publishStatus(const std_msgs::Header& header, const std::string& state,
                     const std::string& reason, const DecodedFrame& decoded,
                     const RegistrationResult& registration,
                     const IntegrationResult& integration,
                     const TimingMetrics& timing,
                     std::size_t registration_points,
                     std::size_t mesh_triangles, bool mesh_limit,
                     std::uint8_t level) {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = level;
    const bool base_publish_failed_closed =
        last_base_output_topology_.reason != "not_run" &&
        !last_base_output_topology_.valid;
    if (base_publish_failed_closed) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    status.name = "local_tsdf_mesh/mapper";
    status.hardware_id = "local_rolling_tsdf";
    status.message =
        state + ":" +
        (base_publish_failed_closed ? "base_output_topology_invalid" : reason);
    addDiagnostic(status, "state", state);
    addDiagnostic(status, "reason", reason);
    addDiagnostic(status, "input_frame", header.frame_id);
    addDiagnostic(status, "expected_input_frame",
                  expected_input_frame_.empty() ? "any" : expected_input_frame_);
    addDiagnostic(status, "raw_points", toString(decoded.raw_points));
    addDiagnostic(status, "finite_points", toString(decoded.finite_points));
    addDiagnostic(status, "invalid_points", toString(decoded.invalid_points));
    addDiagnostic(status, "registration_range_points",
                  toString(decoded.registration_range_points));
    addDiagnostic(status, "tsdf_range_points",
                  toString(decoded.tsdf_range_points));
    addDiagnostic(status, "surface_range_points",
                  toString(decoded.surface_range_points));
    addDiagnostic(status, "registration_range_filtered",
                  toString(decoded.registration_range_filtered));
    addDiagnostic(status, "tsdf_range_filtered",
                  toString(decoded.tsdf_range_filtered));
    addDiagnostic(status, "surface_range_filtered",
                  toString(decoded.surface_range_filtered));
    addDiagnostic(status, "source_topology_points",
                  toString(decoded.source_topology_points));
    addDiagnostic(status, "registration_points", toString(registration_points));
    addDiagnostic(status, "registration_accepted",
                  registration.accepted ? "true" : "false");
    addDiagnostic(status, "registration_converged",
                  registration.quality.converged ? "true" : "false");
    addDiagnostic(status, "integration_accepted",
                  integration.accepted ? "true" : "false");
    addDiagnostic(status, "volume_updated",
                  integration.volume_updated ? "true" : "false");
    addDiagnostic(status, "volume_committed",
                  integration.volume_committed ? "true" : "false");
    addDiagnostic(status, "surface_prepared",
                  integration.surface_prepared ? "true" : "false");
    addDiagnostic(status, "surface_output_accepted",
                  integration.surface_output_accepted ? "true" : "false");
    addDiagnostic(status, "mesh_published_this_frame",
                  integration.mesh_published_this_frame ? "true" : "false");
    addDiagnostic(status, "surface_mode", integration.surface_mode);
    addDiagnostic(status, "surface_output_reason",
                  integration.surface_output_reason);
    addDiagnostic(status, "measured_reason", integration.measured_reason);
    addDiagnostic(status, "measured_capacity_limited",
                  integration.measured_capacity_limited ? "true" : "false");
    addDiagnostic(status, "fitness_score",
                  formatDouble(registration.quality.fitness_score));
    addDiagnostic(status, "inlier_ratio",
                  formatDouble(registration.quality.inlier_ratio));
    addDiagnostic(status, "relative_translation_m",
                  formatDouble(registration.quality.translation));
    addDiagnostic(status, "relative_rotation_deg",
                  formatDouble(registration.quality.rotation_rad *
                               180.0 / 3.14159265358979323846));
    addDiagnostic(status, "motion_prediction_valid",
                  motion_prediction_valid_ ? "true" : "false");
    addDiagnostic(status, "safe_single_frame_fallback",
                  safe_single_frame_fallback_ ? "true" : "false");
    addDiagnostic(status, "prediction_translation_error_m",
                  formatDouble(last_prediction_translation_error_));
    addDiagnostic(status, "prediction_rotation_error_deg",
                  formatDouble(last_prediction_rotation_error_rad_ *
                               180.0 / 3.14159265358979323846));
    addDiagnostic(status, "integrated_rays", toString(integration.integrated_rays));
    addDiagnostic(status, "measured_tsdf_enabled",
                  tsdf_config_.integrate_measured_rays ? "true" : "false");
    addDiagnostic(status, "parallel_support_build",
                  tsdf_config_.parallel_support_build ? "true" : "false");
    addDiagnostic(status, "support_build_parallel",
                  integration.support_build_parallel ? "true" : "false");
    addDiagnostic(status, "parallel_support_validation",
                  parallel_support_validation_ ? "true" : "false");
    addDiagnostic(status, "support_validation_parallel",
                  last_support_validation_parallel_ ? "true" : "false");
    addDiagnostic(status, "support_validation_launch_failed",
                  last_support_validation_launch_failed_ ? "true" : "false");
    addDiagnostic(status, "support_enabled",
                  support_config_.enabled ? "true" : "false");
    addDiagnostic(status, "support_build_surface_rays",
                  support_config_.build_surface_rays ? "true" : "false");
    addDiagnostic(status, "support_build_indexed_mesh",
                  support_config_.build_indexed_mesh ? "true" : "false");
    addDiagnostic(status, "support_reason", integration.support_reason);
    addDiagnostic(status, "support_candidate_budget_limited",
                  integration.support_candidate_budget_limited ? "true"
                                                               : "false");
    addDiagnostic(status, "support_surface_budget_limited",
                  integration.support_surface_budget_limited ? "true"
                                                             : "false");
    addDiagnostic(status, "support_input_points",
                  toString(integration.support_input_points));
    addDiagnostic(status, "support_topology_points",
                  toString(integration.support_topology_points));
    addDiagnostic(status, "support_ring_pairs",
                  toString(integration.support_ring_pairs));
    addDiagnostic(status, "support_rejected_ring_pairs",
                  toString(integration.support_rejected_ring_pairs));
    addDiagnostic(status, "support_rejected_ring_order_pairs",
                  toString(integration.support_rejected_ring_order_pairs));
    addDiagnostic(status, "support_candidate_quads",
                  toString(integration.support_candidate_quads));
    addDiagnostic(status, "support_locally_valid_quads",
                  toString(integration.support_locally_valid_quads));
    addDiagnostic(status, "support_strong_quads",
                  toString(integration.support_strong_quads));
    addDiagnostic(status, "support_accepted_quads",
                  toString(integration.support_accepted_quads));
    addDiagnostic(status, "support_verified_long_quads",
                  toString(integration.support_verified_long_quads));
    addDiagnostic(status, "support_rejected_long_quads",
                  toString(integration.support_rejected_long_quads));
    addDiagnostic(status, "support_rejected_isolated_quads",
                  toString(integration.support_rejected_isolated_quads));
    addDiagnostic(status, "support_candidate_samples",
                  toString(integration.support_candidate_samples));
    addDiagnostic(status, "support_surface_rays",
                  toString(integration.support_surface_rays));
    addDiagnostic(status, "support_integrated_rays",
                  toString(integration.support_integrated_rays));
    addDiagnostic(status, "support_contributed_voxels",
                  toString(integration.support_contributed_voxels));
    addDiagnostic(status, "support_mesh_vertices",
                  toString(integration.support_mesh_vertices));
    addDiagnostic(status, "support_mesh_triangles",
                  toString(integration.support_mesh_triangles));
    addDiagnostic(status, "support_mesh_curve_intervals",
                  toString(integration.support_mesh_curve_intervals));
    addDiagnostic(status, "support_mesh_skipped_curve_intervals",
                  toString(integration.support_mesh_skipped_curve_intervals));
    addDiagnostic(
        status, "support_mesh_skipped_degenerate_intervals",
        toString(integration.support_mesh_skipped_degenerate_intervals));
    addDiagnostic(status, "support_mesh_skipped_sensor_columns",
                  toString(integration.support_mesh_skipped_sensor_columns));
    addDiagnostic(
        status, "support_mesh_output_equivalence_input_triangles",
        toString(integration.support_mesh_output_equivalence_input_triangles));
    addDiagnostic(
        status, "support_mesh_output_equivalence_removed_triangles",
        toString(integration.support_mesh_output_equivalence_removed_triangles));
    addDiagnostic(
        status, "support_mesh_output_equivalence_masked_sensor_columns",
        toString(
            integration.support_mesh_output_equivalence_masked_sensor_columns));
    addDiagnostic(status, "support_mesh_has_first_curve_gap",
                  integration.support_mesh_has_first_curve_gap ? "true"
                                                               : "false");
    addDiagnostic(status, "support_mesh_first_curve_gap_sensor_id",
                  toString(static_cast<unsigned int>(
                      integration.support_mesh_first_curve_gap_sensor_id)));
    addDiagnostic(status, "support_mesh_first_curve_gap_lower_ring",
                  toString(integration.support_mesh_first_curve_gap_lower_ring));
    addDiagnostic(
        status, "support_mesh_first_curve_gap_start_deg",
        formatDouble(integration.support_mesh_first_curve_gap_start_azimuth *
                     180.0 / 3.14159265358979323846));
    addDiagnostic(
        status, "support_mesh_first_curve_gap_end_deg",
        formatDouble(integration.support_mesh_first_curve_gap_end_azimuth *
                     180.0 / 3.14159265358979323846));
    addDiagnostic(status, "support_mesh_first_curve_gap_corner",
                  integration.support_mesh_first_curve_gap_corner);
    addDiagnostic(status, "support_mesh_reason",
                  integration.support_mesh_reason);
    addDiagnostic(status, "support_mesh_budget_limited",
                  integration.support_mesh_budget_limited ? "true" : "false");
    addDiagnostic(status, "support_mesh_accepted",
                  integration.support_mesh_accepted ? "true" : "false");
    addDiagnostic(status, "support_mesh_applied",
                  integration.support_mesh_applied ? "true" : "false");
    addDiagnostic(status, "support_mesh_apply_reason",
                  integration.support_mesh_apply_reason);
    addDiagnostic(status, "contributed_voxels",
                  toString(integration.contributed_voxels));
    addDiagnostic(status, "evicted_frames", toString(integration.evicted_frames));
    addDiagnostic(status, "active_frames", toString(volume_->frameCount()));
    addDiagnostic(status, "active_voxels", toString(volume_->voxelCount()));
    addDiagnostic(status, "mesh_triangles", toString(mesh_triangles));
    addDiagnostic(status, "mesh_triangle_limit_reached",
                  mesh_limit ? "true" : "false");
    addDiagnostic(status, "mesh_cleanup_input_vertices",
                  toString(last_mesh_cleanup_stats_.input_vertices));
    addDiagnostic(status, "mesh_cleanup_merged_vertices",
                  toString(last_mesh_cleanup_stats_.merged_vertices));
    addDiagnostic(status, "mesh_cleanup_input_triangles",
                  toString(last_mesh_cleanup_stats_.input_triangles));
    addDiagnostic(status, "mesh_cleanup_output_triangles",
                  toString(last_mesh_cleanup_stats_.output_triangles));
    addDiagnostic(status, "mesh_cleanup_rejected_total",
                  toString(last_mesh_cleanup_stats_.rejectedTotal()));
    addDiagnostic(status, "base_cleanup_input_triangles",
                  toString(last_mesh_cleanup_stats_.input_triangles));
    addDiagnostic(status, "base_cleanup_output_triangles",
                  toString(last_mesh_cleanup_stats_.output_triangles));
    addDiagnostic(status, "base_cleanup_rejected_total",
                  toString(last_mesh_cleanup_stats_.rejectedTotal()));
    addDiagnostic(status, "mesh_cleanup_rejected_invalid_index",
                  toString(last_mesh_cleanup_stats_.rejected_invalid_index));
    addDiagnostic(status, "mesh_cleanup_rejected_nonfinite",
                  toString(last_mesh_cleanup_stats_.rejected_nonfinite));
    addDiagnostic(status, "mesh_cleanup_rejected_degenerate",
                  toString(last_mesh_cleanup_stats_.rejected_degenerate));
    addDiagnostic(status, "mesh_cleanup_rejected_duplicate",
                  toString(last_mesh_cleanup_stats_.rejected_duplicate));
    addDiagnostic(status, "mesh_cleanup_rejected_large_face",
                  toString(last_mesh_cleanup_stats_.rejected_large_face));
    addDiagnostic(status, "mesh_cleanup_rejected_nonmanifold",
                  toString(last_mesh_cleanup_stats_.rejected_nonmanifold));
    addDiagnostic(
        status, "mesh_cleanup_rejected_nonmanifold_vertex",
        toString(last_mesh_cleanup_stats_.rejected_nonmanifold_vertex));
    addDiagnostic(
        status, "base_cleanup_rejected_equivalent_degenerate",
        toString(last_mesh_cleanup_stats_.rejected_equivalent_degenerate));
    addDiagnostic(
        status, "base_cleanup_rejected_equivalent_duplicate",
        toString(last_mesh_cleanup_stats_.rejected_equivalent_duplicate));
    addDiagnostic(status, "base_cleanup_rejected_equivalent_fan",
                  toString(last_mesh_cleanup_stats_.rejected_equivalent_fan));
    addDiagnostic(status, "mesh_cleanup_rejected_small_component",
                  toString(last_mesh_cleanup_stats_.rejected_small_component));
    addDiagnostic(status, "mesh_components_before_filter",
                  toString(last_mesh_cleanup_stats_.components_before_filter));
    addDiagnostic(status, "mesh_components_after_filter",
                  toString(last_mesh_cleanup_stats_.components_after_filter));
    addDiagnostic(
        status, "mesh_boundary_edges_before_cleanup",
        toString(last_mesh_cleanup_stats_.boundary_edges_before_cleanup));
    addDiagnostic(
        status, "mesh_boundary_edges_after_cleanup",
        toString(last_mesh_cleanup_stats_.boundary_edges_after_cleanup));
    addDiagnostic(
        status, "mesh_nonmanifold_edges_before_cleanup",
        toString(last_mesh_cleanup_stats_.nonmanifold_edges_before_cleanup));
    addDiagnostic(
        status, "mesh_nonmanifold_edges_after_cleanup",
        toString(last_mesh_cleanup_stats_.nonmanifold_edges_after_cleanup));
    addDiagnostic(
        status, "mesh_nonmanifold_vertices_before_cleanup",
        toString(last_mesh_cleanup_stats_.nonmanifold_vertices_before_cleanup));
    addDiagnostic(
        status, "mesh_nonmanifold_vertices_after_cleanup",
        toString(last_mesh_cleanup_stats_.nonmanifold_vertices_after_cleanup));
    addDiagnostic(status, "base_output_topology_valid",
                  last_base_output_topology_.valid ? "true" : "false");
    addDiagnostic(status, "base_publish_failed_closed",
                  base_publish_failed_closed ? "true" : "false");
    addDiagnostic(status, "base_output_topology_reason",
                  last_base_output_topology_.reason);
    addDiagnostic(status, "base_output_nonmanifold_vertices",
                  toString(last_base_output_topology_.nonmanifold_vertices));
    addDiagnostic(status, "support_validation_input_triangles",
                  toString(last_support_validation_stats_.input_triangles));
    addDiagnostic(status, "support_validation_output_triangles",
                  toString(last_support_validation_stats_.output_triangles));
    addDiagnostic(status, "support_validation_rejected_total",
                  toString(last_support_validation_stats_.rejectedTotal()));
    addDiagnostic(
        status, "support_validation_rejected_invalid_index",
        toString(last_support_validation_stats_.rejected_invalid_index));
    addDiagnostic(status, "support_validation_rejected_nonfinite",
                  toString(last_support_validation_stats_.rejected_nonfinite));
    addDiagnostic(
        status, "support_validation_rejected_degenerate",
        toString(last_support_validation_stats_.rejected_degenerate));
    addDiagnostic(status, "support_validation_rejected_duplicate",
                  toString(last_support_validation_stats_.rejected_duplicate));
    addDiagnostic(
        status, "support_validation_rejected_large_face",
        toString(last_support_validation_stats_.rejected_large_face));
    addDiagnostic(
        status, "support_validation_rejected_nonmanifold_vertex",
        toString(last_support_validation_stats_.rejected_nonmanifold_vertex));
    addDiagnostic(
        status, "support_validation_rejected_nonmanifold_edge",
        toString(last_support_validation_stats_.rejected_nonmanifold));
    addDiagnostic(status, "support_output_topology_valid",
                  last_support_output_topology_.valid ? "true" : "false");
    addDiagnostic(status, "support_output_topology_reason",
                  last_support_output_topology_.reason);
    addDiagnostic(status, "combined_topology_valid",
                  last_combined_topology_.valid ? "true" : "false");
    addDiagnostic(status, "combined_topology_reason",
                  last_combined_topology_.reason);
    addDiagnostic(status, "combined_topology_nonmanifold_edges",
                  toString(last_combined_topology_.nonmanifold_edges));
    addDiagnostic(status, "combined_topology_nonmanifold_vertices",
                  toString(last_combined_topology_.nonmanifold_vertices));
    addDiagnostic(status, "last_mesh_stamp_ns",
                  last_mesh_event_stamp_.isZero()
                      ? "0"
                      : toString(last_mesh_event_stamp_.toNSec()));
    const double mesh_age_ms =
        !last_mesh_event_stamp_.isZero() &&
                header.stamp >= last_mesh_event_stamp_
            ? (header.stamp - last_mesh_event_stamp_).toSec() * 1000.0
            : std::numeric_limits<double>::infinity();
    addDiagnostic(status, "mesh_age_ms", formatDouble(mesh_age_ms, 3));
    addDiagnostic(status, "mesh_current_frame_aligned",
                  mesh_triangles > 0 && mesh_age_ms <= 1e-3 ? "true"
                                                           : "false");
    addDiagnostic(status, "consecutive_registration_failures",
                  toString(consecutive_registration_failures_));
    addDiagnostic(status, "registration_failures_total",
                  toString(registration_failures_total_));
    addDiagnostic(status, "integrated_frames_total", toString(integrated_frames_));
    addDiagnostic(status, "surface_frames_total", toString(surface_frames_));
    addDiagnostic(status, "surface_rejected_frames_total",
                  toString(surface_rejected_frames_));
    addDiagnostic(status, "direct_applied_frames_total",
                  toString(direct_applied_frames_));
    addDiagnostic(status, "rejected_frames_total", toString(rejected_frames_));
    addDiagnostic(status, "reset_count", toString(reset_count_));
    addDiagnostic(status, "decode_ms", formatDouble(timing.decode_ms, 3));
    addDiagnostic(status, "registration_ms",
                  formatDouble(timing.registration_ms, 3));
    addDiagnostic(status, "integration_ms",
                  formatDouble(timing.integration_ms, 3));
    addDiagnostic(status, "support_build_ms",
                  formatDouble(integration.support_build_ms, 3));
    addDiagnostic(status, "support_integration_ms",
                  formatDouble(integration.support_integration_ms, 3));
    addDiagnostic(status, "extraction_ms",
                  formatDouble(timing.extraction_ms, 3));
    addDiagnostic(status, "mesh_cleanup_ms",
                  formatDouble(timing.mesh_cleanup_ms, 3));
    addDiagnostic(status, "mesh_base_cleanup_ms",
                  formatDouble(last_mesh_preparation_timings_.base_cleanup_ms,
                               3));
    addDiagnostic(
        status, "mesh_base_equivalent_repair_ms",
        formatDouble(
            last_mesh_preparation_timings_.base_equivalent_repair_ms, 3));
    addDiagnostic(
        status, "mesh_support_validation_ms",
        formatDouble(last_mesh_preparation_timings_.support_validation_ms, 3));
    addDiagnostic(
        status, "mesh_support_validation_wait_ms",
        formatDouble(
            last_mesh_preparation_timings_.support_validation_wait_ms, 3));
    addDiagnostic(status, "mesh_append_ms",
                  formatDouble(last_mesh_preparation_timings_.append_ms, 3));
    addDiagnostic(
        status, "mesh_cross_equivalence_probe_ms",
        formatDouble(
            last_mesh_preparation_timings_.cross_equivalence_probe_ms, 3));
    addDiagnostic(
        status, "mesh_combined_validation_ms",
        formatDouble(last_mesh_preparation_timings_.combined_validation_ms,
                     3));
    addDiagnostic(status, "total_ms", formatDouble(timing.total_ms, 3));
    addDiagnostic(status, "marker_semantics", "full_atomic_snapshot");
    addDiagnostic(status, "marker_coordinate_frame",
                  "last_published_cloud_local_frame");
    array.status.push_back(status);
    status_pub_.publish(array);
  }

  void std_msgsHeaderFallback(const sensor_msgs::PointCloud2& message,
                              const std::string& state,
                              const std::string& reason,
                              const RegistrationResult& registration,
                              const IntegrationResult& integration,
                              const TimingMetrics& timing,
                              std::size_t registration_points,
                              std::size_t mesh_triangles, bool mesh_limit) {
    DecodedFrame decoded;
    decoded.raw_points =
        static_cast<std::size_t>(message.width) * message.height;
    publishStatus(message.header, state, reason, decoded, registration, integration,
                  timing, registration_points, mesh_triangles, mesh_limit,
                  diagnostic_msgs::DiagnosticStatus::ERROR);
  }

  void resetMappingState() {
    volume_->clear();
    initialized_ = false;
    input_frame_id_.clear();
    previous_cloud_.reset();
    reference_from_current_.setIdentity();
    last_relative_transform_.setIdentity();
    motion_prediction_valid_ = false;
    last_prediction_translation_error_ = 0.0;
    last_prediction_rotation_error_rad_ = 0.0;
    consecutive_registration_failures_ = 0;
    have_received_stamp_ = false;
    last_received_stamp_ = ros::Time();
    last_mesh_publish_stamp_ = ros::Time();
    last_mesh_event_stamp_ = ros::Time();
    latest_integrated_header_ = std_msgs::Header();
    last_mesh_triangles_ = 0;
    last_mesh_limit_reached_ = false;
    last_mesh_cleanup_stats_ = MeshCleanupStats();
    last_support_validation_stats_ = MeshCleanupStats();
    last_mesh_preparation_timings_ =
        LayeredMeshPreparationResult::Timings();
    last_support_validation_parallel_ = false;
    last_support_validation_launch_failed_ = false;
    last_base_output_topology_ = MeshTopologyValidationResult();
    last_support_output_topology_ = MeshTopologyValidationResult();
    last_combined_topology_ = MeshTopologyValidationResult();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher status_pub_;

  std::string input_topic_ = "/points_raw";
  std::string mesh_topic_ = "/local_tsdf_mesh/mesh";
  std::string status_topic_ = "/local_tsdf_mesh/status";
  std::string expected_input_frame_;
  RegistrationConfig registration_config_;
  TsdfConfig tsdf_config_;
  ScanStripSupportConfig support_config_;
  MeshExtractionConfig mesh_config_;
  MeshCleanupConfig mesh_cleanup_config_;
  MeshCleanupConfig support_validation_config_;
  std::unique_ptr<FrameRegistrar> registrar_;
  std::unique_ptr<SparseTsdfVolume> volume_;

  double registration_voxel_size_ = 0.20;
  double registration_min_range_ = 0.50;
  double registration_max_range_ = 30.0;
  std::size_t registration_max_points_ = 30000;
  int failures_before_reset_ = 3;
  double mesh_publish_rate_hz_ = 0.0;
  bool parallel_support_validation_ = false;
  double mesh_color_r_ = 0.92;
  double mesh_color_g_ = 0.58;
  double mesh_color_b_ = 0.16;
  double mesh_color_a_ = 0.72;
  bool require_configured_sensor_origin_ = true;
  bool require_lidar_id_ = true;
  bool safe_single_frame_fallback_ = true;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      sensor_origins_;

  bool initialized_ = false;
  bool have_received_stamp_ = false;
  std::string input_frame_id_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
  Eigen::Matrix4f reference_from_current_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f last_relative_transform_ = Eigen::Matrix4f::Identity();
  bool motion_prediction_valid_ = false;
  double last_prediction_translation_error_ = 0.0;
  double last_prediction_rotation_error_rad_ = 0.0;
  ros::Time last_received_stamp_;
  ros::Time last_mesh_publish_stamp_;
  ros::Time last_mesh_event_stamp_;
  std_msgs::Header latest_integrated_header_;
  int consecutive_registration_failures_ = 0;
  std::size_t registration_failures_total_ = 0;
  std::size_t integrated_frames_ = 0;
  std::size_t surface_frames_ = 0;
  std::size_t surface_rejected_frames_ = 0;
  std::size_t direct_applied_frames_ = 0;
  std::size_t rejected_frames_ = 0;
  std::size_t reset_count_ = 0;
  std::size_t last_mesh_triangles_ = 0;
  bool last_mesh_limit_reached_ = false;
  MeshCleanupStats last_mesh_cleanup_stats_;
  MeshCleanupStats last_support_validation_stats_;
  LayeredMeshPreparationResult::Timings last_mesh_preparation_timings_;
  bool last_support_validation_parallel_ = false;
  bool last_support_validation_launch_failed_ = false;
  MeshTopologyValidationResult last_base_output_topology_;
  MeshTopologyValidationResult last_support_output_topology_;
  MeshTopologyValidationResult last_combined_topology_;
};

}  // namespace local_tsdf_mesh

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_tsdf_mesh_node");
  try {
    local_tsdf_mesh::LocalTsdfMeshNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("local_tsdf_mesh initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
