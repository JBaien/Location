#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace local_tsdf_mesh {

struct ScanStripSupportConfig;

struct VoxelKey {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const;
};

struct TsdfConfig {
  // Set false to skip measured-return TSDF integration. The scan-strip
  // builder still consumes the decoded sensor clouds; topology-derived
  // surface rays may still populate the volume when separately enabled. A
  // direct-strip-only mode sets this and build_surface_rays both false.
  bool integrate_measured_rays = true;
  // Build the read-only scan-strip candidate on one bounded worker while the
  // caller constructs the measured-ray contribution.  The worker is joined
  // before support results are copied or any volume state is committed.
  bool parallel_support_build = true;
  float voxel_size = 0.10f;
  float truncation_distance = 0.30f;
  float min_range = 0.50f;
  float max_range = 30.0f;
  float max_weight_per_voxel_per_frame = 4.0f;
  double window_duration_sec = 1.50;
  std::size_t max_window_frames = 15;
  std::size_t max_points_per_frame = 12000;
  std::size_t max_voxels = 500000;
};

struct MeshExtractionConfig {
  float minimum_weight = 1.0f;
  float iso_level = 0.0f;
  std::size_t max_triangles = 200000;
};

struct IndexedMesh {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> triangles;
  bool triangle_limit_reached = false;
};

struct IntegrationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  // `volume_updated` describes the volume instance passed to integrateFrame.
  // A node using a temporary volume sets `volume_committed` only after that
  // candidate becomes the active volume.
  bool volume_updated = false;
  bool volume_committed = false;
  bool surface_prepared = false;
  bool surface_output_accepted = false;
  bool mesh_published_this_frame = false;
  std::string reason = "not_run";
  std::string surface_output_reason = "not_prepared";
  std::string surface_mode = "none";
  std::string measured_reason = "not_run";
  bool measured_capacity_limited = false;
  std::size_t input_points = 0;
  std::size_t sampled_points = 0;
  std::size_t integrated_rays = 0;
  std::size_t contributed_voxels = 0;
  std::size_t evicted_frames = 0;
  std::size_t active_frames = 0;
  std::size_t active_voxels = 0;
  std::string support_reason = "disabled";
  bool support_candidate_budget_limited = false;
  bool support_surface_budget_limited = false;
  std::size_t support_input_points = 0;
  std::size_t support_topology_points = 0;
  std::size_t support_ring_pairs = 0;
  std::size_t support_rejected_ring_pairs = 0;
  std::size_t support_rejected_ring_order_pairs = 0;
  std::size_t support_candidate_quads = 0;
  std::size_t support_locally_valid_quads = 0;
  std::size_t support_strong_quads = 0;
  std::size_t support_accepted_quads = 0;
  std::size_t support_verified_long_quads = 0;
  std::size_t support_rejected_long_quads = 0;
  std::size_t support_rejected_isolated_quads = 0;
  std::size_t support_candidate_samples = 0;
  std::size_t support_surface_rays = 0;
  std::size_t support_integrated_rays = 0;
  std::size_t support_contributed_voxels = 0;
  std::string support_mesh_reason = "disabled";
  std::string support_mesh_apply_reason = "not_published";
  bool support_mesh_budget_limited = false;
  bool support_mesh_accepted = false;
  bool support_mesh_applied = false;
  std::size_t support_mesh_vertices = 0;
  std::size_t support_mesh_triangles = 0;
  std::size_t support_mesh_curve_intervals = 0;
  std::size_t support_mesh_skipped_curve_intervals = 0;
  std::size_t support_mesh_skipped_degenerate_intervals = 0;
  std::size_t support_mesh_skipped_sensor_columns = 0;
  std::size_t support_mesh_output_equivalence_input_triangles = 0;
  std::size_t support_mesh_output_equivalence_removed_triangles = 0;
  std::size_t support_mesh_output_equivalence_masked_sensor_columns = 0;
  bool support_mesh_has_first_curve_gap = false;
  std::uint8_t support_mesh_first_curve_gap_sensor_id = 0;
  std::uint16_t support_mesh_first_curve_gap_lower_ring = 0;
  double support_mesh_first_curve_gap_start_azimuth = 0.0;
  double support_mesh_first_curve_gap_end_azimuth = 0.0;
  std::string support_mesh_first_curve_gap_corner = "none";
  IndexedMesh support_mesh;
  bool support_build_parallel = false;
  double support_build_ms = 0.0;
  double support_integration_ms = 0.0;
};

// A surface return together with the acquisition topology used for bounded
// ray selection.  `azimuth` is the source-LiDAR angle in radians when the
// input provides it.  NaN is permitted and makes the sampler derive a stable
// fallback angle from position-origin instead.
struct TsdfPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TsdfPoint() = default;
  TsdfPoint(const Eigen::Vector3f& value) : position(value) {}

  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  std::uint16_t ring = 0;
  float azimuth = std::numeric_limits<float>::quiet_NaN();

  // True only when both values above came from the source LiDAR packet.  A
  // fallback azimuth reconstructed after multi-LiDAR fusion remains useful
  // for bounded sampling, but it must never be used to connect physical scan
  // lines because each sensor has a different extrinsic rotation.
  bool source_topology_valid = false;
};

// All points and the matching origin are expressed in the volume reference
// frame. Keeping one cloud per physical LiDAR prevents incorrect free-space
// carving when an already-fused scan contains spatially separated sensors.
struct SensorCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint8_t sensor_id = 0;
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  std::vector<TsdfPoint, Eigen::aligned_allocator<TsdfPoint>> points;
};

using SensorClouds =
    std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>>;

struct RaySelection {
  std::size_t sensor_index = 0;
  std::size_t point_index = 0;

  bool operator==(const RaySelection& other) const {
    return sensor_index == other.sensor_index && point_index == other.point_index;
  }
};

// Produces one deterministic progressive ordering across LiDARs, rings and
// one-degree azimuth sectors.  Any budget is a prefix of that ordering, so
// increasing max_rays cannot remove a ray selected by a smaller budget.
std::vector<RaySelection> selectStratifiedRays(
    const SensorClouds& sensors, std::size_t max_rays, float min_range,
    float max_range);

struct MeshAppendResult {
  bool applied = false;
  bool budget_limited = false;
  std::string reason = "not_run";
  std::size_t appended_vertices = 0;
  std::size_t appended_triangles = 0;
};

// Appends one complete indexed mesh after validating every source index and
// the total triangle/index capacity.  Any validation or budget failure leaves
// `destination` unchanged.  Taking `addition` by value lets runtime callers
// move a per-frame support mesh without copying its vertices.
MeshAppendResult appendIndexedMeshAtomic(IndexedMesh addition,
                                         std::size_t max_total_triangles,
                                         IndexedMesh& destination);

class SparseTsdfVolume {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SparseTsdfVolume(const TsdfConfig& config);

  IntegrationResult integrateFrame(
      const SensorClouds& sensors, double stamp_sec);
  IntegrationResult integrateFrame(
      const SensorClouds& sensors, double stamp_sec,
      const ScanStripSupportConfig& support_config);

  IndexedMesh extractMesh(const MeshExtractionConfig& config) const;
  IndexedMesh extractMesh(const MeshExtractionConfig& config,
                          const Eigen::Vector3f& focus) const;

  void clear();
  std::size_t voxelCount() const;
  std::size_t frameCount() const;
  double latestStamp() const;

  // Intended for diagnostics and focused tests.
  bool voxelValue(const VoxelKey& key, float& distance, float& weight) const;
  VoxelKey keyForPoint(const Eigen::Vector3f& point) const;
  Eigen::Vector3f centerForKey(const VoxelKey& key) const;

 private:
  struct Accumulator {
    double weighted_distance_sum = 0.0;
    double weight = 0.0;
  };
  using ContributionMap =
      std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash>;

  struct FrameContribution {
    double stamp_sec = 0.0;
    ContributionMap voxels;
  };

  std::size_t evictExpired(double stamp_sec);
  void removeOldestFrame();
  std::size_t projectedVoxelCount(const ContributionMap& contribution) const;

  TsdfConfig config_;
  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels_;
  std::deque<FrameContribution> frames_;
  double latest_stamp_sec_ = -1.0;
};

}  // namespace local_tsdf_mesh
