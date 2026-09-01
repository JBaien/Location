#pragma once

#include <cstddef>
#include <string>

#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {

// Project acceptance envelope for topology-equivalent output coordinates.
// Builders and the final Marker validator must use the same scale and C++
// llround quantization semantics.
constexpr double kOutputTopologyEquivalenceToleranceM = 1e-4;

// Post-processing bounds for a mesh emitted by the marching-tetra extractor.
// The component test is deliberately conjunctive: a component is removed only
// when its triangle count, accumulated area, and spatial extent are all small.
struct MeshCleanupConfig {
  double minimum_triangle_area = 0.0;
  double maximum_triangle_area = 0.0;
  double maximum_edge_length = 0.0;
  // Vertices whose coordinates quantize to the same cell are treated as one
  // topological vertex. A value of zero disables welding. The extractor can
  // emit the same iso-surface endpoint through several lattice edges, so a
  // very small tolerance is required before edge/component analysis.
  double vertex_merge_tolerance = 0.0;
  std::size_t minimum_component_triangles = 1;
  double maximum_small_component_area = 0.0;
  double maximum_small_component_extent = 0.0;
};

struct MeshCleanupStats {
  std::size_t input_vertices = 0;
  std::size_t input_triangles = 0;
  std::size_t output_vertices = 0;
  std::size_t output_triangles = 0;
  std::size_t merged_vertices = 0;

  std::size_t rejected_invalid_index = 0;
  std::size_t rejected_nonfinite = 0;
  std::size_t rejected_degenerate = 0;
  std::size_t rejected_duplicate = 0;
  std::size_t rejected_large_face = 0;
  std::size_t rejected_nonmanifold = 0;
  std::size_t rejected_nonmanifold_vertex = 0;
  std::size_t rejected_equivalent_degenerate = 0;
  std::size_t rejected_equivalent_duplicate = 0;
  std::size_t rejected_equivalent_fan = 0;
  std::size_t rejected_small_component = 0;

  std::size_t components_before_filter = 0;
  std::size_t components_after_filter = 0;
  std::size_t boundary_edges_before_cleanup = 0;
  std::size_t boundary_edges_after_cleanup = 0;
  std::size_t nonmanifold_edges_before_cleanup = 0;
  std::size_t nonmanifold_edges_after_cleanup = 0;
  std::size_t nonmanifold_vertices_before_cleanup = 0;
  std::size_t nonmanifold_vertices_after_cleanup = 0;

  std::size_t rejectedTotal() const;
};

struct MeshCleanupResult {
  IndexedMesh mesh;
  MeshCleanupStats stats;
};

// Creates conservative bounds from the extractor lattice. A triangle emitted
// inside one voxel cube cannot have an edge longer than sqrt(3) * voxel_size;
// a small tolerance is included for floating-point interpolation.
MeshCleanupConfig marchingTetraCleanupConfig(float voxel_size);

// Geometry bounds for a direct supported-strip mesh. Unlike marching-tetra
// cleanup this configuration does not weld vertices or remove small
// components: the strip builder's indexed topology is authoritative and any
// validation change must make the caller reject the whole addition.
MeshCleanupConfig supportedStripValidationConfig(
    float voxel_size, float maximum_mesh_edge_voxels);

// Returns a compacted mesh whose undirected edge degree is at most two and
// whose used-vertex links are one path or cycle. Edge conflicts are resolved
// in canonical triangle order; at a bow-tie vertex, the incident fan with the
// greatest area (then triangle count, then canonical key) is retained.
MeshCleanupResult cleanupMeshTopology(const IndexedMesh& input,
                                      const MeshCleanupConfig& config);

// Read-only topology check. `vertex_equivalence_tolerance` defines the
// project's acceptance envelope for coordinates that are close enough to be
// treated as one topological point. The wire format itself does not quantize
// or merge vertices; coordinates and indices are never modified here.
struct MeshTopologyValidationResult {
  bool valid = false;
  std::string reason = "not_run";
  std::size_t invalid_indices = 0;
  std::size_t nonfinite_triangles = 0;
  std::size_t degenerate_triangles = 0;
  std::size_t duplicate_triangles = 0;
  std::size_t boundary_edges = 0;
  std::size_t nonmanifold_edges = 0;
  std::size_t nonmanifold_vertices = 0;
};

MeshTopologyValidationResult validateMeshTopology(
    const IndexedMesh& mesh, double vertex_equivalence_tolerance);

// Cleans the marching-tetra base first, then repairs topology at the fixed
// output-equivalence scale by deleting complete faces while preserving every
// retained coordinate. The direct strip remains an all-or-nothing read-only
// validation, and the union is appended atomically. Any support failure
// returns the repaired base unchanged. When requested, support validation may
// run on one joined worker while the caller cleans the independent base mesh;
// launch failure transparently uses the same serial validator.
struct LayeredMeshPreparationResult {
  struct Timings {
    double base_cleanup_ms = 0.0;
    double base_equivalent_repair_ms = 0.0;
    double support_validation_ms = 0.0;
    // In parallel mode support_validation_ms remains the worker's own elapsed
    // validation time. This field reports only how long the caller blocked
    // while joining that already-running worker.
    double support_validation_wait_ms = 0.0;
    double append_ms = 0.0;
    double cross_equivalence_probe_ms = 0.0;
    double combined_validation_ms = 0.0;
  } timings;
  IndexedMesh mesh;
  MeshCleanupStats base_cleanup;
  MeshCleanupStats support_validation;
  MeshTopologyValidationResult base_output_topology;
  MeshTopologyValidationResult support_output_topology;
  MeshTopologyValidationResult combined_topology;
  bool support_applied = false;
  bool support_budget_limited = false;
  bool support_validation_parallel = false;
  bool support_validation_launch_failed = false;
  std::string support_reason = "not_run";
};

LayeredMeshPreparationResult prepareLayeredMeshForPublish(
    const IndexedMesh& raw_base, const IndexedMesh& raw_support,
    std::size_t max_total_triangles,
    const MeshCleanupConfig& base_cleanup_config,
    const MeshCleanupConfig& support_validation_config,
    double output_vertex_equivalence_tolerance,
    bool parallel_support_validation = false);

}  // namespace local_tsdf_mesh
