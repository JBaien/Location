#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "local_tsdf_mesh/mesh_cleanup.h"
#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {
namespace {

MeshCleanupConfig permissiveConfig() {
  MeshCleanupConfig config;
  config.minimum_triangle_area = 1e-12;
  config.maximum_triangle_area = 1000.0;
  config.maximum_edge_length = 1000.0;
  config.minimum_component_triangles = 1U;
  return config;
}

std::set<std::vector<int>> canonicalTriangles(const IndexedMesh& mesh) {
  std::set<std::vector<int>> result;
  for (const auto& triangle : mesh.triangles) {
    std::vector<int> key{triangle.x(), triangle.y(), triangle.z()};
    std::sort(key.begin(), key.end());
    result.insert(std::move(key));
  }
  return result;
}

std::multiset<std::array<float, 9>> canonicalTriangleGeometry(
    const IndexedMesh& mesh) {
  std::multiset<std::array<float, 9>> result;
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    std::array<std::array<float, 3>, 3> points;
    for (int corner = 0; corner < 3; ++corner) {
      const Eigen::Vector3f& point =
          mesh.vertices[static_cast<std::size_t>(triangle[corner])];
      points[static_cast<std::size_t>(corner)] =
          {{point.x(), point.y(), point.z()}};
    }
    std::sort(points.begin(), points.end());
    std::array<float, 9> key;
    for (std::size_t point = 0U; point < points.size(); ++point) {
      for (std::size_t axis = 0U; axis < points[point].size(); ++axis) {
        key[point * 3U + axis] = points[point][axis];
      }
    }
    result.insert(key);
  }
  return result;
}

void expectMeshesEqual(const IndexedMesh& left, const IndexedMesh& right) {
  ASSERT_EQ(left.vertices.size(), right.vertices.size());
  ASSERT_EQ(left.triangles.size(), right.triangles.size());
  EXPECT_EQ(left.triangle_limit_reached, right.triangle_limit_reached);
  for (std::size_t index = 0; index < left.vertices.size(); ++index) {
    for (int axis = 0; axis < 3; ++axis) {
      std::uint32_t left_bits = 0U;
      std::uint32_t right_bits = 0U;
      std::memcpy(&left_bits, &left.vertices[index][axis], sizeof(left_bits));
      std::memcpy(&right_bits, &right.vertices[index][axis],
                  sizeof(right_bits));
      EXPECT_EQ(left_bits, right_bits);
    }
  }
  for (std::size_t index = 0; index < left.triangles.size(); ++index) {
    EXPECT_EQ(left.triangles[index].x(), right.triangles[index].x());
    EXPECT_EQ(left.triangles[index].y(), right.triangles[index].y());
    EXPECT_EQ(left.triangles[index].z(), right.triangles[index].z());
  }
}

void expectCleanupStatsEqual(const MeshCleanupStats& left,
                             const MeshCleanupStats& right) {
#define EXPECT_STATS_FIELD(field) EXPECT_EQ(left.field, right.field)
  EXPECT_STATS_FIELD(input_vertices);
  EXPECT_STATS_FIELD(input_triangles);
  EXPECT_STATS_FIELD(output_vertices);
  EXPECT_STATS_FIELD(output_triangles);
  EXPECT_STATS_FIELD(merged_vertices);
  EXPECT_STATS_FIELD(rejected_invalid_index);
  EXPECT_STATS_FIELD(rejected_nonfinite);
  EXPECT_STATS_FIELD(rejected_degenerate);
  EXPECT_STATS_FIELD(rejected_duplicate);
  EXPECT_STATS_FIELD(rejected_large_face);
  EXPECT_STATS_FIELD(rejected_nonmanifold);
  EXPECT_STATS_FIELD(rejected_nonmanifold_vertex);
  EXPECT_STATS_FIELD(rejected_equivalent_degenerate);
  EXPECT_STATS_FIELD(rejected_equivalent_duplicate);
  EXPECT_STATS_FIELD(rejected_equivalent_fan);
  EXPECT_STATS_FIELD(rejected_small_component);
  EXPECT_STATS_FIELD(components_before_filter);
  EXPECT_STATS_FIELD(components_after_filter);
  EXPECT_STATS_FIELD(boundary_edges_before_cleanup);
  EXPECT_STATS_FIELD(boundary_edges_after_cleanup);
  EXPECT_STATS_FIELD(nonmanifold_edges_before_cleanup);
  EXPECT_STATS_FIELD(nonmanifold_edges_after_cleanup);
  EXPECT_STATS_FIELD(nonmanifold_vertices_before_cleanup);
  EXPECT_STATS_FIELD(nonmanifold_vertices_after_cleanup);
#undef EXPECT_STATS_FIELD
}

void expectTopologyEqual(const MeshTopologyValidationResult& left,
                         const MeshTopologyValidationResult& right) {
  EXPECT_EQ(left.valid, right.valid);
  EXPECT_EQ(left.reason, right.reason);
  EXPECT_EQ(left.invalid_indices, right.invalid_indices);
  EXPECT_EQ(left.nonfinite_triangles, right.nonfinite_triangles);
  EXPECT_EQ(left.degenerate_triangles, right.degenerate_triangles);
  EXPECT_EQ(left.duplicate_triangles, right.duplicate_triangles);
  EXPECT_EQ(left.boundary_edges, right.boundary_edges);
  EXPECT_EQ(left.nonmanifold_edges, right.nonmanifold_edges);
  EXPECT_EQ(left.nonmanifold_vertices, right.nonmanifold_vertices);
}

void expectLayeredResultsEquivalent(
    const LayeredMeshPreparationResult& serial,
    const LayeredMeshPreparationResult& parallel) {
  expectMeshesEqual(serial.mesh, parallel.mesh);
  expectCleanupStatsEqual(serial.base_cleanup, parallel.base_cleanup);
  expectCleanupStatsEqual(serial.support_validation,
                          parallel.support_validation);
  expectTopologyEqual(serial.base_output_topology,
                      parallel.base_output_topology);
  expectTopologyEqual(serial.support_output_topology,
                      parallel.support_output_topology);
  expectTopologyEqual(serial.combined_topology,
                      parallel.combined_topology);
  EXPECT_EQ(serial.support_applied, parallel.support_applied);
  EXPECT_EQ(serial.support_budget_limited,
            parallel.support_budget_limited);
  EXPECT_EQ(serial.support_reason, parallel.support_reason);
}

TEST(MeshCleanupTest, RejectsMalformedDegenerateAndDuplicateTriangles) {
  IndexedMesh input;
  input.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 1.0f, 0.0f),
      Eigen::Vector3f(2.0f, 0.0f, 0.0f),
      Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)};
  input.triangles = {
      Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(2, 1, 0),
      Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 1, 3),
      Eigen::Vector3i(0, 1, 4), Eigen::Vector3i(0, 1, 99)};

  const MeshCleanupResult cleaned =
      cleanupMeshTopology(input, permissiveConfig());
  ASSERT_EQ(cleaned.mesh.triangles.size(), 1U);
  EXPECT_EQ(cleaned.stats.rejected_duplicate, 1U);
  EXPECT_EQ(cleaned.stats.rejected_degenerate, 2U);
  EXPECT_EQ(cleaned.stats.rejected_nonfinite, 1U);
  EXPECT_EQ(cleaned.stats.rejected_invalid_index, 1U);
  EXPECT_EQ(cleaned.stats.rejectedTotal(), 5U);
  EXPECT_EQ(cleaned.stats.boundary_edges_after_cleanup, 3U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_after_cleanup, 0U);
}

TEST(MeshCleanupTest,
     TopologyCountsDuplicateFacesAndHighDegreeEdgesExactly) {
  IndexedMesh duplicated;
  duplicated.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                         Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                         Eigen::Vector3f(0.0f, 1.0f, 0.0f)};
  duplicated.triangles = {Eigen::Vector3i(0, 1, 2),
                          Eigen::Vector3i(2, 1, 0)};
  const MeshTopologyValidationResult duplicate_topology =
      validateMeshTopology(duplicated, 0.0);
  EXPECT_EQ(duplicate_topology.duplicate_triangles, 1U);
  EXPECT_EQ(duplicate_topology.boundary_edges, 3U);
  EXPECT_EQ(duplicate_topology.nonmanifold_edges, 0U);
  EXPECT_EQ(duplicate_topology.nonmanifold_vertices, 0U);

  IndexedMesh high_degree;
  high_degree.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                          Eigen::Vector3f(1.0f, 0.0f, 0.0f)};
  constexpr int kIncidentFaces = 300;
  high_degree.vertices.reserve(static_cast<std::size_t>(kIncidentFaces + 2));
  high_degree.triangles.reserve(static_cast<std::size_t>(kIncidentFaces));
  for (int face = 0; face < kIncidentFaces; ++face) {
    const float angle = static_cast<float>(face) * 0.01f;
    high_degree.vertices.emplace_back(0.5f, std::cos(angle),
                                      std::sin(angle));
    high_degree.triangles.emplace_back(0, 1, face + 2);
  }
  const MeshTopologyValidationResult high_degree_topology =
      validateMeshTopology(high_degree, 0.0);
  EXPECT_EQ(high_degree_topology.degenerate_triangles, 0U);
  EXPECT_EQ(high_degree_topology.duplicate_triangles, 0U);
  EXPECT_EQ(high_degree_topology.boundary_edges,
            static_cast<std::size_t>(2 * kIncidentFaces));
  EXPECT_EQ(high_degree_topology.nonmanifold_edges, 1U);
  EXPECT_EQ(high_degree_topology.nonmanifold_vertices, 2U);
  EXPECT_EQ(high_degree_topology.reason, "nonmanifold_edge");
}

TEST(MeshCleanupTest, ResolvesNonmanifoldEdgesDeterministically) {
  IndexedMesh ordered;
  ordered.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 1.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.0f, 1.0f),
      Eigen::Vector3f(0.0f, -1.0f, 0.0f)};
  ordered.triangles = {Eigen::Vector3i(0, 1, 2),
                       Eigen::Vector3i(0, 1, 3),
                       Eigen::Vector3i(0, 1, 4)};
  IndexedMesh shuffled = ordered;
  std::reverse(shuffled.triangles.begin(), shuffled.triangles.end());

  const MeshCleanupResult first =
      cleanupMeshTopology(ordered, permissiveConfig());
  const MeshCleanupResult second =
      cleanupMeshTopology(shuffled, permissiveConfig());
  ASSERT_EQ(first.mesh.triangles.size(), 2U);
  ASSERT_EQ(second.mesh.triangles.size(), 2U);
  EXPECT_EQ(canonicalTriangles(first.mesh), canonicalTriangles(second.mesh));
  EXPECT_EQ(first.stats.nonmanifold_edges_before_cleanup, 1U);
  EXPECT_EQ(first.stats.rejected_nonmanifold, 1U);
  EXPECT_EQ(first.stats.nonmanifold_edges_after_cleanup, 0U);
  EXPECT_EQ(first.stats.boundary_edges_after_cleanup, 4U);
}

TEST(MeshCleanupTest, WeldsCoincidentVerticesBeforeTopologyChecks) {
  IndexedMesh input;
  input.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 1.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.0f, 1.0f),
      Eigen::Vector3f(1e-6f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f + 1e-6f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, -1.0f, 0.0f)};
  input.triangles = {Eigen::Vector3i(0, 1, 2),
                     Eigen::Vector3i(4, 5, 3),
                     Eigen::Vector3i(0, 1, 6)};
  MeshCleanupConfig config = permissiveConfig();
  config.vertex_merge_tolerance = 1e-4;

  const MeshCleanupResult cleaned = cleanupMeshTopology(input, config);
  EXPECT_EQ(cleaned.stats.merged_vertices, 2U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_before_cleanup, 1U);
  EXPECT_EQ(cleaned.stats.rejected_nonmanifold, 1U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_after_cleanup, 0U);
  EXPECT_EQ(cleaned.mesh.triangles.size(), 2U);
}

TEST(MeshCleanupTest, SmallComponentFilterPreservesSparseLargeWall) {
  IndexedMesh input;
  input.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(2.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 2.0f, 0.0f),
      Eigen::Vector3f(2.0f, 2.0f, 0.0f),
      Eigen::Vector3f(10.0f, 0.0f, 0.0f),
      Eigen::Vector3f(10.01f, 0.0f, 0.0f),
      Eigen::Vector3f(10.0f, 0.01f, 0.0f)};
  input.triangles = {Eigen::Vector3i(0, 1, 3),
                     Eigen::Vector3i(0, 3, 2),
                     Eigen::Vector3i(4, 5, 6)};
  MeshCleanupConfig config = permissiveConfig();
  config.minimum_component_triangles = 3U;
  config.maximum_small_component_area = 0.01;
  config.maximum_small_component_extent = 0.1;

  const MeshCleanupResult cleaned = cleanupMeshTopology(input, config);
  EXPECT_EQ(cleaned.mesh.triangles.size(), 2U);
  EXPECT_EQ(cleaned.stats.components_before_filter, 2U);
  EXPECT_EQ(cleaned.stats.components_after_filter, 1U);
  EXPECT_EQ(cleaned.stats.rejected_small_component, 1U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_after_cleanup, 0U);
}

TEST(MeshCleanupTest, RejectsAbnormalLargeFaceWithoutRemovingMainWall) {
  IndexedMesh input;
  input.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 1.0f, 0.0f),
      Eigen::Vector3f(1.0f, 1.0f, 0.0f),
      Eigen::Vector3f(2.0f, 0.0f, 0.0f),
      Eigen::Vector3f(2.0f, 1.0f, 0.0f),
      Eigen::Vector3f(10.0f, 0.0f, 0.0f),
      Eigen::Vector3f(20.0f, 0.0f, 0.0f),
      Eigen::Vector3f(10.0f, 10.0f, 0.0f)};
  input.triangles = {
      Eigen::Vector3i(0, 1, 3), Eigen::Vector3i(0, 3, 2),
      Eigen::Vector3i(1, 4, 5), Eigen::Vector3i(1, 5, 3),
      Eigen::Vector3i(6, 7, 8)};
  MeshCleanupConfig config = permissiveConfig();
  config.maximum_edge_length = 1.5;
  config.maximum_triangle_area = 1.0;
  config.minimum_component_triangles = 3U;
  config.maximum_small_component_area = 0.1;
  config.maximum_small_component_extent = 0.5;

  const MeshCleanupResult cleaned = cleanupMeshTopology(input, config);
  EXPECT_EQ(cleaned.stats.rejected_large_face, 1U);
  EXPECT_EQ(cleaned.mesh.triangles.size(), 4U);
  EXPECT_EQ(cleaned.stats.components_after_filter, 1U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_after_cleanup, 0U);
}

TEST(MeshCleanupTest, CleansExtractedPlaneWithoutDiscardingMainSurface) {
  TsdfConfig tsdf;
  tsdf.voxel_size = 0.10f;
  tsdf.truncation_distance = 0.30f;
  tsdf.min_range = 0.10f;
  tsdf.max_range = 10.0f;
  tsdf.max_points_per_frame = 10000U;
  tsdf.max_voxels = 100000U;
  SparseTsdfVolume volume(tsdf);
  std::vector<SensorCloud, Eigen::aligned_allocator<SensorCloud>> sensors;
  for (int y = -4; y <= 4; ++y) {
    for (int z = -4; z <= 4; ++z) {
      SensorCloud sensor;
      sensor.origin = Eigen::Vector3f(0.05f, y * 0.10f + 0.05f,
                                      z * 0.10f + 0.05f);
      sensor.points.push_back(Eigen::Vector3f(
          1.05f, y * 0.10f + 0.05f, z * 0.10f + 0.05f));
      sensors.push_back(std::move(sensor));
    }
  }
  ASSERT_TRUE(volume.integrateFrame(sensors, 1.0).accepted);
  MeshExtractionConfig extraction;
  extraction.minimum_weight = 0.5f;
  extraction.max_triangles = 10000U;
  const IndexedMesh raw = volume.extractMesh(extraction);
  ASSERT_FALSE(raw.triangles.empty());

  const MeshCleanupResult cleaned =
      cleanupMeshTopology(raw, marchingTetraCleanupConfig(tsdf.voxel_size));
  EXPECT_GT(cleaned.mesh.triangles.size(), raw.triangles.size() / 2U);
  EXPECT_EQ(cleaned.stats.rejected_large_face, 0U);
  EXPECT_EQ(cleaned.stats.nonmanifold_edges_after_cleanup, 0U);
  EXPECT_EQ(cleaned.mesh.triangle_limit_reached, raw.triangle_limit_reached);
  for (const auto& triangle : cleaned.mesh.triangles) {
    EXPECT_GE(triangle.minCoeff(), 0);
    EXPECT_LT(static_cast<std::size_t>(triangle.maxCoeff()),
              cleaned.mesh.vertices.size());
  }
}

TEST(MeshCleanupTest, RejectsInvalidConfiguration) {
  IndexedMesh input;
  MeshCleanupConfig config = permissiveConfig();
  config.maximum_edge_length = 0.0;
  EXPECT_THROW(cleanupMeshTopology(input, config), std::invalid_argument);
  EXPECT_THROW(marchingTetraCleanupConfig(0.0f), std::invalid_argument);
}

TEST(MeshCleanupTest, RepairsBowTieByKeepingLargestFanDeterministically) {
  IndexedMesh ordered;
  ordered.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(2.0f, 0.0f, 0.0f),
      Eigen::Vector3f(2.0f, 2.0f, 0.0f),
      Eigen::Vector3f(0.0f, 2.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.2f, 0.0f),
      Eigen::Vector3f(-0.2f, 0.0f, 0.0f)};
  ordered.triangles = {Eigen::Vector3i(0, 1, 2),
                       Eigen::Vector3i(0, 2, 3),
                       Eigen::Vector3i(0, 4, 5)};
  IndexedMesh permuted = ordered;
  std::reverse(permuted.triangles.begin(), permuted.triangles.end());

  const MeshCleanupResult first =
      cleanupMeshTopology(ordered, permissiveConfig());
  const MeshCleanupResult second =
      cleanupMeshTopology(permuted, permissiveConfig());
  EXPECT_EQ(first.stats.nonmanifold_edges_before_cleanup, 0U);
  EXPECT_EQ(first.stats.nonmanifold_vertices_before_cleanup, 1U);
  EXPECT_EQ(first.stats.rejected_nonmanifold_vertex, 1U);
  EXPECT_EQ(first.stats.nonmanifold_vertices_after_cleanup, 0U);
  EXPECT_EQ(first.mesh.triangles.size(), 2U);
  EXPECT_EQ(canonicalTriangles(first.mesh), canonicalTriangles(second.mesh));
  const MeshTopologyValidationResult validation =
      validateMeshTopology(first.mesh, 0.0);
  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_EQ(validation.nonmanifold_edges, 0U);
  EXPECT_EQ(validation.nonmanifold_vertices, 0U);
}

TEST(MeshCleanupTest, DirectStripUsesItsOwnEdgeAndAreaContract) {
  IndexedMesh triangle;
  const float height = std::sqrt(3.0f) * 0.2f;
  triangle.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                       Eigen::Vector3f(0.4f, 0.0f, 0.0f),
                       Eigen::Vector3f(0.2f, height, 0.0f)};
  triangle.triangles = {Eigen::Vector3i(0, 1, 2)};

  const MeshCleanupResult base =
      cleanupMeshTopology(triangle, marchingTetraCleanupConfig(0.15f));
  EXPECT_EQ(base.stats.rejected_large_face, 1U);
  EXPECT_TRUE(base.mesh.triangles.empty());

  const MeshCleanupResult support = cleanupMeshTopology(
      triangle, supportedStripValidationConfig(0.15f, 2.9f));
  EXPECT_EQ(support.stats.rejectedTotal(), 0U);
  ASSERT_EQ(support.mesh.triangles.size(), 1U);
  EXPECT_TRUE(validateMeshTopology(support.mesh, 0.0).valid);
}

TEST(MeshCleanupTest, InvalidSupportRollsBackToBitIdenticalCleanedBase) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 1.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh support;
  support.vertices = {
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.2f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.2f, 0.0f),
      Eigen::Vector3f(0.8f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, -0.2f, 0.0f)};
  support.triangles = {Eigen::Vector3i(0, 1, 2),
                       Eigen::Vector3i(0, 3, 4)};
  const MeshCleanupConfig base_config = permissiveConfig();
  const MeshCleanupConfig support_config =
      supportedStripValidationConfig(0.15f, 2.9f);
  const IndexedMesh expected_base =
      cleanupMeshTopology(base, base_config).mesh;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(base, support, 10U, base_config,
                                   support_config, 1e-4);
  EXPECT_FALSE(prepared.support_applied);
  EXPECT_EQ(prepared.support_reason, "support_validation_changed_mesh");
  EXPECT_GT(prepared.support_validation.rejected_nonmanifold_vertex, 0U);
  expectMeshesEqual(prepared.mesh, expected_base);
}

TEST(MeshCleanupTest, EquivalentSupportCollapseRollsBackWholeAddition) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 1.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh support;
  support.vertices = {
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.00004f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.2f, 0.0f),
      Eigen::Vector3f(2.0f, 0.0f, 0.0f),
      Eigen::Vector3f(2.2f, 0.0f, 0.0f),
      Eigen::Vector3f(2.0f, 0.2f, 0.0f)};
  support.triangles = {Eigen::Vector3i(0, 1, 2),
                       Eigen::Vector3i(3, 4, 5)};
  const MeshCleanupConfig base_config = permissiveConfig();
  const IndexedMesh expected_base =
      cleanupMeshTopology(base, base_config).mesh;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 10U, base_config,
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  EXPECT_FALSE(prepared.support_applied);
  EXPECT_EQ(prepared.support_reason, "support_validation_changed_mesh");
  EXPECT_EQ(prepared.support_output_topology.degenerate_triangles, 1U);
  // The first raw triangle is geometrically valid before applying the 0.1 mm
  // output-equivalence contract. Geometry statistics and topology rejection
  // remain independent even though they are accumulated in one traversal.
  EXPECT_EQ(prepared.support_validation.rejected_degenerate, 1U);
  EXPECT_EQ(prepared.support_validation.output_triangles, 2U);
  expectMeshesEqual(prepared.mesh, expected_base);
}

TEST(MeshCleanupTest, CrossSourcePointContactRollsBackWholeSupport) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh support;
  support.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                      Eigen::Vector3f(0.0f, 0.0f, 0.2f),
                      Eigen::Vector3f(-0.2f, 0.0f, 0.0f)};
  support.triangles = {Eigen::Vector3i(0, 1, 2)};
  const MeshCleanupConfig base_config = permissiveConfig();
  const MeshCleanupConfig support_config =
      supportedStripValidationConfig(0.15f, 2.9f);
  const IndexedMesh expected_base =
      cleanupMeshTopology(base, base_config).mesh;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(base, support, 2U, base_config,
                                   support_config, 1e-4);
  EXPECT_FALSE(prepared.support_applied);
  EXPECT_FALSE(prepared.combined_topology.valid);
  EXPECT_EQ(prepared.combined_topology.nonmanifold_vertices, 1U);
  EXPECT_EQ(prepared.support_reason, "cross_source_topology_conflict");
  expectMeshesEqual(prepared.mesh, expected_base);
}

TEST(MeshCleanupTest, LayeredCapacityIsCheckedAfterBaseCleanup) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 0.0f),
                   Eigen::Vector3f(2.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(2.01f, 0.0f, 0.0f),
                   Eigen::Vector3f(2.0f, 0.01f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2),
                    Eigen::Vector3i(3, 4, 5)};
  MeshCleanupConfig base_config = permissiveConfig();
  base_config.minimum_component_triangles = 2U;
  base_config.maximum_small_component_area = 0.001;
  base_config.maximum_small_component_extent = 0.1;
  IndexedMesh support;
  support.vertices = {Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                      Eigen::Vector3f(0.2f, 0.0f, 1.0f),
                      Eigen::Vector3f(0.0f, 0.2f, 1.0f)};
  support.triangles = {Eigen::Vector3i(0, 1, 2)};
  const MeshCleanupConfig support_config =
      supportedStripValidationConfig(0.15f, 2.9f);

  const LayeredMeshPreparationResult fits =
      prepareLayeredMeshForPublish(base, support, 2U, base_config,
                                   support_config, 1e-4);
  ASSERT_TRUE(fits.support_applied) << fits.support_reason;
  EXPECT_EQ(fits.base_cleanup.rejected_small_component, 1U);
  EXPECT_EQ(fits.mesh.triangles.size(), 2U);

  const LayeredMeshPreparationResult limited =
      prepareLayeredMeshForPublish(base, support, 1U, base_config,
                                   support_config, 1e-4);
  EXPECT_FALSE(limited.support_applied);
  EXPECT_TRUE(limited.support_budget_limited);
  EXPECT_EQ(limited.support_reason, "total_triangle_capacity");
  EXPECT_EQ(limited.mesh.triangles.size(), 1U);
}

TEST(MeshCleanupTest, EquivalentBowTieKeepsOneDeterministicFan) {
  IndexedMesh base;
  base.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.2f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.2f, 0.0f),
      // Distinct to the base cleanup, but equivalent at the fixed 0.1 mm
      // output contract.  The two otherwise-disjoint faces become a bow-tie.
      Eigen::Vector3f(0.00004f, 0.0f, 0.0f),
      Eigen::Vector3f(-0.2f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, -0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2),
                    Eigen::Vector3i(3, 4, 5)};
  const IndexedMesh support;
  MeshCleanupConfig base_config = permissiveConfig();
  base_config.vertex_merge_tolerance = 0.0;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 10U, base_config,
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  EXPECT_TRUE(prepared.base_output_topology.valid);
  EXPECT_EQ(prepared.base_cleanup.rejected_equivalent_fan, 1U);
  EXPECT_EQ(prepared.support_reason, "empty_addition");
  EXPECT_EQ(prepared.mesh.triangles.size(), 1U);
  EXPECT_TRUE(validateMeshTopology(prepared.mesh, 1e-4).valid);
}

TEST(MeshCleanupTest,
     EquivalentCollapseDeletesOnlyFaceWithoutMovingCoordinates) {
  IndexedMesh base;
  base.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.00004f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.2f, 0.0f),
      Eigen::Vector3f(1.0f, 0.0f, 0.0f),
      Eigen::Vector3f(1.2f, 0.0f, 0.0f),
      Eigen::Vector3f(1.0f, 0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2),
                    Eigen::Vector3i(3, 4, 5)};
  const IndexedMesh support;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 10U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  ASSERT_TRUE(prepared.base_output_topology.valid)
      << prepared.base_output_topology.reason;
  EXPECT_EQ(prepared.base_cleanup.rejected_equivalent_degenerate, 1U);
  ASSERT_EQ(prepared.mesh.vertices.size(), 3U);
  ASSERT_EQ(prepared.mesh.triangles.size(), 1U);
  for (std::size_t index = 0U; index < 3U; ++index) {
    EXPECT_TRUE(prepared.mesh.vertices[index].isApprox(
        base.vertices[index + 3U], 0.0f));
  }
  EXPECT_TRUE(validateMeshTopology(prepared.mesh, 1e-4).valid);

  const std::array<std::size_t, 6> new_to_old{{5U, 1U, 3U, 0U, 4U, 2U}};
  std::array<int, 6> old_to_new;
  IndexedMesh permuted;
  for (std::size_t index = 0U; index < new_to_old.size(); ++index) {
    permuted.vertices.push_back(base.vertices[new_to_old[index]]);
    old_to_new[new_to_old[index]] = static_cast<int>(index);
  }
  for (auto triangle = base.triangles.rbegin();
       triangle != base.triangles.rend(); ++triangle) {
    permuted.triangles.emplace_back(
        old_to_new[static_cast<std::size_t>(triangle->x())],
        old_to_new[static_cast<std::size_t>(triangle->y())],
        old_to_new[static_cast<std::size_t>(triangle->z())]);
  }
  const LayeredMeshPreparationResult shuffled =
      prepareLayeredMeshForPublish(
          permuted, support, 10U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  ASSERT_TRUE(shuffled.base_output_topology.valid)
      << shuffled.base_output_topology.reason;
  EXPECT_EQ(shuffled.base_cleanup.rejected_equivalent_degenerate, 1U);
  EXPECT_EQ(canonicalTriangleGeometry(prepared.mesh),
            canonicalTriangleGeometry(shuffled.mesh));
}

TEST(MeshCleanupTest, EquivalentDuplicateKeepsCanonicalOriginalGeometry) {
  IndexedMesh base;
  base.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.2f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.2f, 0.0f),
      Eigen::Vector3f(0.00004f, 0.00004f, 0.00004f),
      Eigen::Vector3f(0.20004f, 0.00004f, 0.00004f),
      Eigen::Vector3f(0.00004f, 0.20004f, 0.00004f)};
  base.triangles = {Eigen::Vector3i(3, 4, 5),
                    Eigen::Vector3i(0, 1, 2)};

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, IndexedMesh(), 10U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  ASSERT_TRUE(prepared.base_output_topology.valid)
      << prepared.base_output_topology.reason;
  EXPECT_EQ(prepared.base_cleanup.rejected_equivalent_duplicate, 1U);
  ASSERT_EQ(prepared.mesh.triangles.size(), 1U);
  ASSERT_EQ(prepared.mesh.vertices.size(), 3U);
  EXPECT_TRUE(prepared.mesh.vertices[0].isApprox(base.vertices[0], 0.0f));
  EXPECT_TRUE(prepared.mesh.vertices[1].isApprox(base.vertices[1], 0.0f));
  EXPECT_TRUE(prepared.mesh.vertices[2].isApprox(base.vertices[2], 0.0f));
}

TEST(MeshCleanupTest, EmptyLimitedSupportPreservesBudgetReason) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh support;
  support.triangle_limit_reached = true;

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 10U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  EXPECT_FALSE(prepared.support_applied);
  EXPECT_TRUE(prepared.support_budget_limited);
  EXPECT_EQ(prepared.support_reason, "support_triangle_limit");
  expectMeshesEqual(prepared.mesh,
                    cleanupMeshTopology(base, permissiveConfig()).mesh);
}

TEST(MeshCleanupTest, AcceptedSupportIsAppendedWithoutCanonicalReordering) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 1.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 1.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};

  IndexedMesh support;
  support.vertices = {Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                      Eigen::Vector3f(1.2f, 0.0f, 0.0f),
                      Eigen::Vector3f(1.2f, 0.2f, 0.0f),
                      Eigen::Vector3f(1.0f, 0.2f, 0.0f)};
  // Deliberately reverse the cleanup's canonical face order.
  support.triangles = {Eigen::Vector3i(0, 2, 3),
                       Eigen::Vector3i(0, 1, 2)};

  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 3U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), 1e-4);
  ASSERT_TRUE(prepared.support_applied) << prepared.support_reason;
  ASSERT_EQ(prepared.mesh.triangles.size(), 3U);
  const int offset = 3;
  for (std::size_t index = 0; index < support.triangles.size(); ++index) {
    EXPECT_EQ(prepared.mesh.triangles[index + 1U],
              support.triangles[index] + Eigen::Vector3i::Constant(offset));
  }
  for (std::size_t index = 0; index < support.vertices.size(); ++index) {
    EXPECT_TRUE(prepared.mesh.vertices[index + 3U].isApprox(
        support.vertices[index], 0.0f));
  }
}

TEST(MeshCleanupTest,
     ParallelSupportValidationMatchesSerialResultsExactly) {
  IndexedMesh base;
  base.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.2f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.2f, 0.0f),
      Eigen::Vector3f(0.00004f, 0.0f, 0.0f),
      Eigen::Vector3f(-0.2f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, -0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2),
                    Eigen::Vector3i(3, 4, 5)};
  IndexedMesh valid_support;
  valid_support.vertices = {Eigen::Vector3f(2.0f, 0.0f, 0.0f),
                            Eigen::Vector3f(2.2f, 0.0f, 0.0f),
                            Eigen::Vector3f(2.0f, 0.2f, 0.0f),
                            Eigen::Vector3f(2.2f, 0.2f, 0.0f)};
  valid_support.triangles = {Eigen::Vector3i(0, 1, 2),
                             Eigen::Vector3i(1, 3, 2)};
  const MeshCleanupConfig base_config = permissiveConfig();
  const MeshCleanupConfig direct_config =
      supportedStripValidationConfig(0.15f, 2.9f);
  const auto compare = [&](const IndexedMesh& support,
                           const MeshCleanupConfig& support_config,
                           std::size_t triangle_limit,
                           bool expect_parallel_launch) {
    const LayeredMeshPreparationResult serial =
        prepareLayeredMeshForPublish(base, support, triangle_limit,
                                     base_config, support_config, 1e-4,
                                     false);
    const LayeredMeshPreparationResult parallel =
        prepareLayeredMeshForPublish(base, support, triangle_limit,
                                     base_config, support_config, 1e-4,
                                     true);
    expectLayeredResultsEquivalent(serial, parallel);
    EXPECT_FALSE(serial.support_validation_parallel);
    EXPECT_FALSE(serial.support_validation_launch_failed);
    EXPECT_EQ(parallel.support_validation_parallel,
              expect_parallel_launch);
    EXPECT_FALSE(parallel.support_validation_launch_failed);
    EXPECT_GE(parallel.timings.support_validation_wait_ms, 0.0);
  };

  compare(valid_support, direct_config, 10U, true);

  IndexedMesh invalid_support;
  invalid_support.vertices = {
      Eigen::Vector3f(3.0f, 0.0f, 0.0f),
      Eigen::Vector3f(3.2f, 0.0f, 0.0f),
      Eigen::Vector3f(3.0f, 0.2f, 0.0f),
      Eigen::Vector3f(2.8f, 0.0f, 0.0f),
      Eigen::Vector3f(3.0f, -0.2f, 0.0f)};
  invalid_support.triangles = {Eigen::Vector3i(0, 1, 2),
                               Eigen::Vector3i(0, 3, 4)};
  compare(invalid_support, direct_config, 10U, true);

  IndexedMesh cross_contact_support;
  cross_contact_support.vertices = {
      Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(0.0f, 0.0f, 0.2f),
      Eigen::Vector3f(-0.2f, 0.0f, 0.0f)};
  cross_contact_support.triangles = {Eigen::Vector3i(0, 1, 2)};
  compare(cross_contact_support, direct_config, 10U, true);
  compare(valid_support, direct_config, 1U, true);

  MeshCleanupConfig cleanup_contract = permissiveConfig();
  cleanup_contract.vertex_merge_tolerance = 1e-6;
  compare(valid_support, cleanup_contract, 10U, true);

  IndexedMesh limited_support = valid_support;
  limited_support.triangle_limit_reached = true;
  compare(limited_support, direct_config, 10U, false);
  compare(IndexedMesh(), direct_config, 10U, false);
}

TEST(MeshCleanupTest,
     ParallelSupportValidationPreservesWorkerExceptionContract) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.2f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.0f, 0.2f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh support;
  support.vertices = {Eigen::Vector3f(2.0f, 0.0f, 0.0f),
                      Eigen::Vector3f(2.2f, 0.0f, 0.0f),
                      Eigen::Vector3f(2.0f, 0.2f, 0.0f)};
  support.triangles = {Eigen::Vector3i(0, 1, 2)};
  MeshCleanupConfig invalid_support_config =
      supportedStripValidationConfig(0.15f, 2.9f);
  invalid_support_config.maximum_edge_length = 0.0;
  const auto exception_message = [&](bool parallel) {
    try {
      (void)prepareLayeredMeshForPublish(
          base, support, 10U, permissiveConfig(), invalid_support_config,
          1e-4, parallel);
    } catch (const std::invalid_argument& error) {
      return std::string(error.what());
    }
    return std::string();
  };
  const std::string serial_message = exception_message(false);
  const std::string parallel_message = exception_message(true);
  EXPECT_FALSE(serial_message.empty());
  EXPECT_EQ(serial_message, parallel_message);

  const auto base_exception_message = [&](bool parallel) {
    try {
      (void)prepareLayeredMeshForPublish(
          base, support, 10U, permissiveConfig(),
          supportedStripValidationConfig(0.15f, 2.9f), -1.0, parallel);
    } catch (const std::invalid_argument& error) {
      return std::string(error.what());
    }
    return std::string();
  };
  const std::string serial_base_message = base_exception_message(false);
  const std::string parallel_base_message = base_exception_message(true);
  EXPECT_FALSE(serial_base_message.empty());
  EXPECT_EQ(serial_base_message, parallel_base_message);
}

// Kept disabled so default unit runs stay fast.  Run explicitly when changing
// the layered validators:
//   --gtest_also_run_disabled_tests --gtest_filter=*H2ScaleMicroBenchmark
TEST(MeshCleanupTest, DISABLED_H2ScaleMicroBenchmark) {
  constexpr int kColumns = 350;
  constexpr int kRows = 200;
  IndexedMesh support;
  support.vertices.reserve(
      static_cast<std::size_t>((kColumns + 1) * (kRows + 1)));
  support.triangles.reserve(
      static_cast<std::size_t>(2 * kColumns * kRows));
  for (int row = 0; row <= kRows; ++row) {
    for (int column = 0; column <= kColumns; ++column) {
      support.vertices.emplace_back(0.2f * static_cast<float>(column),
                                    0.2f * static_cast<float>(row), 0.0f);
    }
  }
  for (int row = 0; row < kRows; ++row) {
    for (int column = 0; column < kColumns; ++column) {
      const int lower = row * (kColumns + 1) + column;
      const int upper = lower + kColumns + 1;
      support.triangles.emplace_back(lower, lower + 1, upper + 1);
      support.triangles.emplace_back(lower, upper + 1, upper);
    }
  }
  constexpr int kBaseColumns = 100;
  constexpr int kBaseRows = 100;
  IndexedMesh base;
  base.vertices.reserve(static_cast<std::size_t>(
      (kBaseColumns + 1) * (kBaseRows + 1)));
  base.triangles.reserve(
      static_cast<std::size_t>(2 * kBaseColumns * kBaseRows));
  for (int row = 0; row <= kBaseRows; ++row) {
    for (int column = 0; column <= kBaseColumns; ++column) {
      base.vertices.emplace_back(0.1f * static_cast<float>(column),
                                 0.1f * static_cast<float>(row), 1.0f);
    }
  }
  for (int row = 0; row < kBaseRows; ++row) {
    for (int column = 0; column < kBaseColumns; ++column) {
      const int lower = row * (kBaseColumns + 1) + column;
      const int upper = lower + kBaseColumns + 1;
      base.triangles.emplace_back(lower, lower + 1, upper + 1);
      base.triangles.emplace_back(lower, upper + 1, upper);
    }
  }
  // Force the runtime-equivalence path with two individually manifold halves
  // whose seam coordinates differ by 20 um. The fixed 0.1 mm topology joins
  // the seam without moving either half's coordinates.
  constexpr int kSeamColumn = kBaseColumns / 2;
  std::vector<int> seam_duplicate(static_cast<std::size_t>(kBaseRows + 1));
  for (int row = 0; row <= kBaseRows; ++row) {
    const int original = row * (kBaseColumns + 1) + kSeamColumn;
    Eigen::Vector3f duplicate =
        base.vertices[static_cast<std::size_t>(original)];
    duplicate.z() += 0.00002f;
    seam_duplicate[static_cast<std::size_t>(row)] =
        static_cast<int>(base.vertices.size());
    base.vertices.push_back(duplicate);
  }
  for (std::size_t index = 0U; index < base.triangles.size(); ++index) {
    const int cell_column =
        static_cast<int>((index / 2U) % static_cast<std::size_t>(kBaseColumns));
    if (cell_column < kSeamColumn) {
      continue;
    }
    Eigen::Vector3i& triangle = base.triangles[index];
    for (int corner = 0; corner < 3; ++corner) {
      const int vertex_column = triangle[corner] % (kBaseColumns + 1);
      if (vertex_column == kSeamColumn) {
        const int row = triangle[corner] / (kBaseColumns + 1);
        triangle[corner] = seam_duplicate[static_cast<std::size_t>(row)];
      }
    }
  }

  using Clock = std::chrono::steady_clock;
  const MeshCleanupConfig support_config =
      supportedStripValidationConfig(0.15f, 2.9f);
  const Clock::time_point cleanup_start = Clock::now();
  const MeshCleanupResult checked =
      cleanupMeshTopology(support, support_config);
  const Clock::time_point cleanup_end = Clock::now();
  const MeshTopologyValidationResult topology =
      validateMeshTopology(support, 1e-4);
  const Clock::time_point topology_end = Clock::now();
  const LayeredMeshPreparationResult prepared =
      prepareLayeredMeshForPublish(
          base, support, 200000U, marchingTetraCleanupConfig(0.15f),
          support_config, 1e-4);
  const Clock::time_point prepare_end = Clock::now();
  const auto elapsed_ms = [](Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };
  std::cout << "H2_SCALE_MESH vertices=" << support.vertices.size()
            << " triangles=" << support.triangles.size()
            << " base_vertices=" << base.vertices.size()
            << " base_triangles=" << base.triangles.size()
            << " cleanup_ms=" << elapsed_ms(cleanup_start, cleanup_end)
            << " topology_ms=" << elapsed_ms(cleanup_end, topology_end)
            << " prepare_ms=" << elapsed_ms(topology_end, prepare_end)
            << " base_cleanup_ms=" << prepared.timings.base_cleanup_ms
            << " base_equiv_ms="
            << prepared.timings.base_equivalent_repair_ms
            << " support_validation_ms="
            << prepared.timings.support_validation_ms
            << " cross_probe_ms="
            << prepared.timings.cross_equivalence_probe_ms
            << std::endl;
  EXPECT_EQ(checked.stats.rejectedTotal(), 0U);
  EXPECT_TRUE(topology.valid) << topology.reason;
  EXPECT_TRUE(prepared.support_applied) << prepared.support_reason;
  EXPECT_EQ(prepared.mesh.triangles.size(),
            support.triangles.size() + base.triangles.size());
}

}  // namespace
}  // namespace local_tsdf_mesh

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
