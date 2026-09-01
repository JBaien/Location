#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {

// Conservative topology gates for filling the two-dimensional space between
// adjacent physical scan rings.  Angles are degrees and distances are metres.
struct ScanStripSupportConfig {
  bool enabled = false;
  float voxel_size = 0.15f;
  float min_range = 8.0f;
  float max_range = 30.0f;

  std::size_t max_surface_cells = 36000U;
  std::size_t max_candidate_samples = 300000U;
  float surface_spacing_voxels = 0.75f;
  float integration_band_voxels = 1.50f;
  float strong_ray_weight = 0.50f;
  float verified_long_ray_weight = 0.25f;
  float maximum_weight_per_voxel = 1.0f;

  // Preserve the original virtual-ray augmentation by default.  Experiments
  // can disable this relatively expensive raster path while still building
  // the indexed strip mesh from the same accepted quad runs.
  bool build_surface_rays = true;

  // Optional direct indexed output.  Runtime configurations can enable it for
  // bag A/B independently of virtual-ray integration.  Triangle edges are
  // bounded by `maximum_mesh_edge_voxels * voxel_size`.
  bool build_indexed_mesh = false;
  std::size_t max_mesh_vertices = 200000U;
  std::size_t max_mesh_triangles = 400000U;
  float maximum_mesh_edge_voxels = 1.0f;
  float mesh_weld_tolerance_voxels = 1e-4f;

  float minimum_ring_angle_deg = 0.20f;
  float maximum_ring_angle_deg = 5.0f;
  float maximum_ring_order_direction_angle_deg = 45.0f;
  float minimum_match_angle_deg = 0.12f;
  float maximum_match_angle_deg = 0.30f;
  float match_step_factor = 1.50f;
  float minimum_gap_angle_deg = 0.35f;
  float maximum_gap_angle_deg = 0.65f;
  float gap_step_factor = 2.60f;

  float maximum_along_log_range_delta = 0.05f;
  float maximum_along_edge_base_m = 0.30f;
  float maximum_along_edge_range_ratio = 0.04f;

  float strong_cross_log_range_delta = 0.025f;
  float strong_cross_edge_base_m = 0.60f;
  float strong_cross_edge_range_ratio = 0.055f;
  float maximum_cross_log_range_delta = 0.25f;
  float maximum_cross_edge_base_m = 1.00f;
  float maximum_cross_edge_range_ratio = 0.25f;
  float maximum_cross_edge_absolute_m = 2.50f;

  float maximum_quad_normal_angle_deg = 25.0f;
  float maximum_neighbor_normal_angle_deg = 15.0f;
  float maximum_opposite_edge_length_ratio = 1.80f;
  float maximum_planarity_base_m = 0.15f;
  float maximum_planarity_edge_ratio = 0.08f;
  float maximum_neighbor_plane_edge_ratio = 0.05f;
  float maximum_run_cross_edge_ratio = 1.60f;
  std::size_t minimum_run_quads = 2U;
};

struct SupportedSurfaceRay {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t sensor_index = 0;
  std::uint8_t sensor_id = 0;
  std::uint16_t lower_ring = 0;
  float azimuth = 0.0f;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  bool verified_long_span = false;
};

struct ScanStripSupportStats {
  std::size_t input_points = 0;
  std::size_t topology_points = 0;
  std::size_t ring_pairs = 0;
  std::size_t rejected_ring_pairs = 0;
  std::size_t rejected_ring_order_pairs = 0;
  std::size_t candidate_quads = 0;
  std::size_t locally_valid_quads = 0;
  std::size_t strong_quads = 0;
  std::size_t verified_long_quads = 0;
  std::size_t rejected_long_quads = 0;
  std::size_t accepted_run_quads = 0;
  std::size_t rejected_isolated_quads = 0;
  std::size_t candidate_samples = 0;
  std::size_t unique_surface_cells = 0;
  std::size_t selected_surface_rays = 0;
};

struct SupportedStripMesh {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  bool budget_limited = false;
  std::string reason = "disabled";
  std::size_t curve_intervals = 0;
  std::size_t skipped_curve_intervals = 0;
  std::size_t skipped_degenerate_intervals = 0;
  // Number of unique (sensor, azimuth-start, azimuth-end) columns removed
  // from every ring pair after any one pair failed curve resampling or mesh
  // welding.  Whole-column removal prevents complementary gaps from leaving
  // bow-tie/vertex-link non-manifold topology.
  std::size_t skipped_sensor_columns = 0;
  // Output-equivalence repair accounting is kept separate from the aggregate
  // skipped count above.  It measures the raw zipper mesh before the 0.1 mm
  // project acceptance envelope is applied, and the exact whole-column cost
  // of making that output topology valid.
  std::size_t output_equivalence_input_triangles = 0;
  std::size_t output_equivalence_removed_triangles = 0;
  std::size_t output_equivalence_masked_sensor_columns = 0;
  bool has_first_curve_gap = false;
  std::uint8_t first_curve_gap_sensor_id = 0;
  std::uint16_t first_curve_gap_lower_ring = 0;
  double first_curve_gap_start_azimuth = 0.0;
  double first_curve_gap_end_azimuth = 0.0;
  std::string first_curve_gap_corner = "none";
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      vertices;
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
      triangles;
};

struct ScanStripSupportResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  bool candidate_budget_limited = false;
  bool surface_budget_limited = false;
  std::string reason = "not_run";
  ScanStripSupportStats stats;
  std::vector<SupportedSurfaceRay,
              Eigen::aligned_allocator<SupportedSurfaceRay>> rays;
  SupportedStripMesh mesh;
};

// Assigns source topology only when the raw PointCloud2 values are genuinely
// present and valid.  Individually valid fields remain useful to ordinary ray
// sampling, but the returned topology flag requires both; reconstructed
// fallback values must never be upgraded to source topology.
bool assignSourceTopologyFields(bool has_ring, double raw_ring,
                                bool has_azimuth, double raw_azimuth,
                                TsdfPoint& point);

// Builds deterministic, bounded virtual surface returns.  This function does
// not mutate a TSDF volume and is therefore independently testable.  It fails
// closed when source topology or three-ring evidence is unavailable.
ScanStripSupportResult buildSupportedScanStrips(
    const SensorClouds& sensors, const ScanStripSupportConfig& config);

}  // namespace local_tsdf_mesh
