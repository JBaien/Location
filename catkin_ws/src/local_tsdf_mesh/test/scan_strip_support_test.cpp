#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "local_tsdf_mesh/mesh_cleanup.h"
#include "local_tsdf_mesh/scan_strip_support.h"

// Compile one private copy into this test translation unit so the chained
// output-topology repair loop can be exercised without exposing test-only
// seams in the public header. Rename the two public definitions and the only
// test-helper name collision; all production calls below still link the core
// library after these macros are undefined.
#define assignSourceTopologyFields assignSourceTopologyFieldsForInternalTest
#define buildSupportedScanStrips buildSupportedScanStripsForInternalTest
#define kPi kScanStripSupportImplementationPi
#define radians scanStripSupportImplementationRadians
#include "../src/scan_strip_support.cpp"
#undef radians
#undef kPi
#undef buildSupportedScanStrips
#undef assignSourceTopologyFields

namespace local_tsdf_mesh {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float radians(float degrees) { return degrees * kPi / 180.0f; }

TsdfPoint topologyPoint(const Eigen::Vector3f& position, std::uint16_t ring,
                        float azimuth) {
  TsdfPoint point(position);
  point.ring = ring;
  point.azimuth = azimuth;
  point.source_topology_valid = true;
  return point;
}

Eigen::Vector3f rayDirection(float azimuth, float elevation) {
  const float cosine = std::cos(elevation);
  return Eigen::Vector3f(cosine * std::cos(azimuth),
                         cosine * std::sin(azimuth), std::sin(elevation));
}

SensorCloud makeWallSensor(const std::vector<std::uint16_t>& rings,
                           const std::vector<float>& elevations_deg,
                           const std::vector<float>& azimuths_deg,
                           float wall_x = 10.0f) {
  EXPECT_EQ(rings.size(), elevations_deg.size());
  SensorCloud sensor;
  sensor.sensor_id = 0;
  for (std::size_t ring_index = 0; ring_index < rings.size(); ++ring_index) {
    const float elevation = radians(elevations_deg[ring_index]);
    for (const float azimuth_deg : azimuths_deg) {
      const float azimuth = radians(azimuth_deg);
      const Eigen::Vector3f direction = rayDirection(azimuth, elevation);
      const float distance = wall_x / direction.x();
      sensor.points.push_back(topologyPoint(direction * distance,
                                            rings[ring_index], azimuth));
    }
  }
  return sensor;
}

SensorCloud makeGroundSensor(const std::vector<std::uint16_t>& rings,
                             const std::vector<float>& elevations_deg,
                             const std::vector<float>& azimuths_deg,
                             float plane_z = -2.0f) {
  EXPECT_EQ(rings.size(), elevations_deg.size());
  SensorCloud sensor;
  sensor.sensor_id = 0;
  for (std::size_t ring_index = 0; ring_index < rings.size(); ++ring_index) {
    const float elevation = radians(elevations_deg[ring_index]);
    for (const float azimuth_deg : azimuths_deg) {
      const float azimuth = radians(azimuth_deg);
      const Eigen::Vector3f direction = rayDirection(azimuth, elevation);
      const float distance = plane_z / direction.z();
      sensor.points.push_back(topologyPoint(direction * distance,
                                            rings[ring_index], azimuth));
    }
  }
  return sensor;
}

SensorCloud makeSphereSensor(const std::vector<std::uint16_t>& rings,
                             const std::vector<float>& elevations_deg,
                             const std::vector<float>& azimuths_deg,
                             float range = 10.0f) {
  EXPECT_EQ(rings.size(), elevations_deg.size());
  SensorCloud sensor;
  sensor.sensor_id = 0;
  for (std::size_t ring_index = 0; ring_index < rings.size(); ++ring_index) {
    const float elevation = radians(elevations_deg[ring_index]);
    for (const float azimuth_deg : azimuths_deg) {
      const float azimuth = radians(azimuth_deg);
      sensor.points.push_back(topologyPoint(
          rayDirection(azimuth, elevation) * range, rings[ring_index],
          azimuth));
    }
  }
  return sensor;
}

SensorCloud makeUnevenSphereSensor(
    const std::vector<std::vector<float>>& ring_azimuths_deg,
    const std::vector<float>& elevations_deg, float range) {
  EXPECT_EQ(ring_azimuths_deg.size(), elevations_deg.size());
  SensorCloud sensor;
  sensor.sensor_id = 0U;
  for (std::size_t ring = 0; ring < ring_azimuths_deg.size(); ++ring) {
    const float elevation = radians(elevations_deg[ring]);
    for (const float azimuth_deg : ring_azimuths_deg[ring]) {
      const float azimuth = radians(azimuth_deg);
      sensor.points.push_back(topologyPoint(
          rayDirection(azimuth, elevation) * range,
          static_cast<std::uint16_t>(ring), azimuth));
    }
  }
  return sensor;
}

ScanStripSupportConfig testConfig() {
  ScanStripSupportConfig config;
  config.enabled = true;
  config.voxel_size = 0.15f;
  config.min_range = 1.0f;
  config.max_range = 30.0f;
  config.max_surface_cells = 10000U;
  config.max_candidate_samples = 100000U;
  return config;
}

using MeshEdge = std::pair<int, int>;

std::map<MeshEdge, std::size_t> meshEdgeCounts(
    const SupportedStripMesh& mesh) {
  std::map<MeshEdge, std::size_t> counts;
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    const std::array<int, 3> indices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (std::size_t edge = 0; edge < 3U; ++edge) {
      int first = indices[edge];
      int second = indices[(edge + 1U) % 3U];
      if (second < first) {
        std::swap(first, second);
      }
      ++counts[MeshEdge(first, second)];
    }
  }
  return counts;
}

std::size_t boundaryComponentCount(const SupportedStripMesh& mesh) {
  const std::map<MeshEdge, std::size_t> counts = meshEdgeCounts(mesh);
  std::map<int, std::vector<int>> adjacency;
  for (const auto& edge : counts) {
    if (edge.second != 1U) {
      continue;
    }
    adjacency[edge.first.first].push_back(edge.first.second);
    adjacency[edge.first.second].push_back(edge.first.first);
  }
  std::set<int> visited;
  std::size_t components = 0U;
  for (const auto& vertex : adjacency) {
    EXPECT_EQ(vertex.second.size(), 2U) << "boundary vertex=" << vertex.first;
    if (visited.count(vertex.first) != 0U) {
      continue;
    }
    ++components;
    std::vector<int> pending(1U, vertex.first);
    visited.insert(vertex.first);
    while (!pending.empty()) {
      const int current = pending.back();
      pending.pop_back();
      for (const int neighbor : adjacency[current]) {
        if (visited.insert(neighbor).second) {
          pending.push_back(neighbor);
        }
      }
    }
  }
  return components;
}

void expectBoundedManifoldEdges(const SupportedStripMesh& mesh,
                                float maximum_edge) {
  for (const auto& edge : meshEdgeCounts(mesh)) {
    EXPECT_LE(edge.second, 2U);
  }
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    ASSERT_GE(triangle.x(), 0);
    ASSERT_GE(triangle.y(), 0);
    ASSERT_GE(triangle.z(), 0);
    ASSERT_LT(static_cast<std::size_t>(triangle.x()), mesh.vertices.size());
    ASSERT_LT(static_cast<std::size_t>(triangle.y()), mesh.vertices.size());
    ASSERT_LT(static_cast<std::size_t>(triangle.z()), mesh.vertices.size());
    const Eigen::Vector3f& a = mesh.vertices[triangle.x()];
    const Eigen::Vector3f& b = mesh.vertices[triangle.y()];
    const Eigen::Vector3f& c = mesh.vertices[triangle.z()];
    EXPECT_LE((a - b).norm(), maximum_edge + 1e-4f);
    EXPECT_LE((b - c).norm(), maximum_edge + 1e-4f);
    EXPECT_LE((c - a).norm(), maximum_edge + 1e-4f);
  }
}

void expectConsistentWindingTowardOrigin(const SupportedStripMesh& mesh,
                                         const Eigen::Vector3f& origin) {
  std::map<MeshEdge, int> directed_balance;
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    const Eigen::Vector3f& a = mesh.vertices[triangle.x()];
    const Eigen::Vector3f& b = mesh.vertices[triangle.y()];
    const Eigen::Vector3f& c = mesh.vertices[triangle.z()];
    const Eigen::Vector3f centroid = (a + b + c) / 3.0f;
    EXPECT_GE((b - a).cross(c - a).dot(origin - centroid), -1e-6f);
    const std::array<int, 3> indices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (std::size_t edge = 0; edge < 3U; ++edge) {
      const int from = indices[edge];
      const int to = indices[(edge + 1U) % 3U];
      const MeshEdge key = from < to ? MeshEdge(from, to) : MeshEdge(to, from);
      directed_balance[key] += from < to ? 1 : -1;
    }
  }
  const std::map<MeshEdge, std::size_t> counts = meshEdgeCounts(mesh);
  for (const auto& edge : counts) {
    if (edge.second == 2U) {
      EXPECT_EQ(directed_balance[edge.first], 0) << "edge="
                                                 << edge.first.first << ','
                                                 << edge.first.second;
    }
  }
}

std::size_t boundaryEdgesContainingAnotherVertex(
    const SupportedStripMesh& mesh) {
  std::size_t count = 0U;
  for (const auto& edge : meshEdgeCounts(mesh)) {
    if (edge.second != 1U) {
      continue;
    }
    const Eigen::Vector3f& start = mesh.vertices[edge.first.first];
    const Eigen::Vector3f& end = mesh.vertices[edge.first.second];
    const Eigen::Vector3f segment = end - start;
    const float length_squared = segment.squaredNorm();
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
      if (static_cast<int>(index) == edge.first.first ||
          static_cast<int>(index) == edge.first.second) {
        continue;
      }
      const float fraction =
          (mesh.vertices[index] - start).dot(segment) / length_squared;
      if (fraction <= 1e-4f || fraction >= 1.0f - 1e-4f) {
        continue;
      }
      const Eigen::Vector3f projected = start + fraction * segment;
      if ((mesh.vertices[index] - projected).norm() <= 2e-5f) {
        ++count;
        break;
      }
    }
  }
  return count;
}

void expectVertexLinksManifold(const SupportedStripMesh& mesh) {
  std::map<int, std::vector<MeshEdge>> links;
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    const std::array<int, 3> vertices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (std::size_t corner = 0; corner < vertices.size(); ++corner) {
      links[vertices[corner]].push_back(
          MeshEdge(vertices[(corner + 1U) % 3U],
                   vertices[(corner + 2U) % 3U]));
    }
  }
  for (const auto& link : links) {
    std::map<int, std::set<int>> adjacency;
    for (const MeshEdge& edge : link.second) {
      adjacency[edge.first].insert(edge.second);
      adjacency[edge.second].insert(edge.first);
    }
    ASSERT_FALSE(adjacency.empty());
    std::size_t degree_one = 0U;
    for (const auto& vertex : adjacency) {
      EXPECT_LE(vertex.second.size(), 2U) << "mesh vertex=" << link.first;
      if (vertex.second.size() == 1U) {
        ++degree_one;
      }
    }
    EXPECT_TRUE(degree_one == 0U || degree_one == 2U)
        << "mesh vertex=" << link.first;
    std::set<int> visited;
    std::vector<int> pending(1U, adjacency.begin()->first);
    visited.insert(adjacency.begin()->first);
    while (!pending.empty()) {
      const int current = pending.back();
      pending.pop_back();
      for (const int neighbor : adjacency[current]) {
        if (visited.insert(neighbor).second) {
          pending.push_back(neighbor);
        }
      }
    }
    EXPECT_EQ(visited.size(), adjacency.size()) << "mesh vertex=" << link.first;
  }
}

void expectNoAzimuthEdgeWiderThan(const SupportedStripMesh& mesh,
                                  float maximum_degrees) {
  for (const auto& edge : meshEdgeCounts(mesh)) {
    const Eigen::Vector3f& first = mesh.vertices[edge.first.first];
    const Eigen::Vector3f& second = mesh.vertices[edge.first.second];
    float delta = std::fabs(std::atan2(first.y(), first.x()) -
                            std::atan2(second.y(), second.x()));
    delta = std::min(delta, 2.0f * kPi - delta);
    EXPECT_LE(delta, radians(maximum_degrees) + 1e-5f);
  }
}

void expectMeshesExactlyEqual(const SupportedStripMesh& first,
                              const SupportedStripMesh& second) {
  ASSERT_EQ(first.vertices.size(), second.vertices.size());
  ASSERT_EQ(first.triangles.size(), second.triangles.size());
  for (std::size_t index = 0; index < first.vertices.size(); ++index) {
    EXPECT_TRUE(first.vertices[index].isApprox(second.vertices[index], 0.0f))
        << "vertex=" << index;
  }
  for (std::size_t index = 0; index < first.triangles.size(); ++index) {
    EXPECT_EQ(first.triangles[index].x(), second.triangles[index].x())
        << "triangle=" << index;
    EXPECT_EQ(first.triangles[index].y(), second.triangles[index].y())
        << "triangle=" << index;
    EXPECT_EQ(first.triangles[index].z(), second.triangles[index].z())
        << "triangle=" << index;
  }
}

void expectSupportResultsExactlyEqual(const ScanStripSupportResult& first,
                                      const ScanStripSupportResult& second) {
  EXPECT_EQ(first.accepted, second.accepted);
  EXPECT_EQ(first.candidate_budget_limited,
            second.candidate_budget_limited);
  EXPECT_EQ(first.surface_budget_limited, second.surface_budget_limited);
  EXPECT_EQ(first.reason, second.reason);

  EXPECT_EQ(first.stats.input_points, second.stats.input_points);
  EXPECT_EQ(first.stats.topology_points, second.stats.topology_points);
  EXPECT_EQ(first.stats.ring_pairs, second.stats.ring_pairs);
  EXPECT_EQ(first.stats.rejected_ring_pairs,
            second.stats.rejected_ring_pairs);
  EXPECT_EQ(first.stats.rejected_ring_order_pairs,
            second.stats.rejected_ring_order_pairs);
  EXPECT_EQ(first.stats.candidate_quads, second.stats.candidate_quads);
  EXPECT_EQ(first.stats.locally_valid_quads,
            second.stats.locally_valid_quads);
  EXPECT_EQ(first.stats.strong_quads, second.stats.strong_quads);
  EXPECT_EQ(first.stats.verified_long_quads,
            second.stats.verified_long_quads);
  EXPECT_EQ(first.stats.rejected_long_quads,
            second.stats.rejected_long_quads);
  EXPECT_EQ(first.stats.accepted_run_quads,
            second.stats.accepted_run_quads);
  EXPECT_EQ(first.stats.rejected_isolated_quads,
            second.stats.rejected_isolated_quads);
  EXPECT_EQ(first.stats.candidate_samples, second.stats.candidate_samples);
  EXPECT_EQ(first.stats.unique_surface_cells,
            second.stats.unique_surface_cells);
  EXPECT_EQ(first.stats.selected_surface_rays,
            second.stats.selected_surface_rays);

  ASSERT_EQ(first.rays.size(), second.rays.size());
  for (std::size_t index = 0U; index < first.rays.size(); ++index) {
    const SupportedSurfaceRay& left = first.rays[index];
    const SupportedSurfaceRay& right = second.rays[index];
    EXPECT_EQ(left.sensor_index, right.sensor_index) << "ray=" << index;
    EXPECT_EQ(left.sensor_id, right.sensor_id) << "ray=" << index;
    EXPECT_EQ(left.lower_ring, right.lower_ring) << "ray=" << index;
    EXPECT_FLOAT_EQ(left.azimuth, right.azimuth) << "ray=" << index;
    EXPECT_TRUE(left.position.isApprox(right.position, 0.0f))
        << "ray=" << index;
    EXPECT_EQ(left.verified_long_span, right.verified_long_span)
        << "ray=" << index;
  }

  EXPECT_EQ(first.mesh.accepted, second.mesh.accepted);
  EXPECT_EQ(first.mesh.budget_limited, second.mesh.budget_limited);
  EXPECT_EQ(first.mesh.reason, second.mesh.reason);
  EXPECT_EQ(first.mesh.curve_intervals, second.mesh.curve_intervals);
  EXPECT_EQ(first.mesh.skipped_curve_intervals,
            second.mesh.skipped_curve_intervals);
  EXPECT_EQ(first.mesh.skipped_degenerate_intervals,
            second.mesh.skipped_degenerate_intervals);
  EXPECT_EQ(first.mesh.skipped_sensor_columns,
            second.mesh.skipped_sensor_columns);
  EXPECT_EQ(first.mesh.output_equivalence_input_triangles,
            second.mesh.output_equivalence_input_triangles);
  EXPECT_EQ(first.mesh.output_equivalence_removed_triangles,
            second.mesh.output_equivalence_removed_triangles);
  EXPECT_EQ(first.mesh.output_equivalence_masked_sensor_columns,
            second.mesh.output_equivalence_masked_sensor_columns);
  EXPECT_EQ(first.mesh.has_first_curve_gap,
            second.mesh.has_first_curve_gap);
  EXPECT_EQ(first.mesh.first_curve_gap_sensor_id,
            second.mesh.first_curve_gap_sensor_id);
  EXPECT_EQ(first.mesh.first_curve_gap_lower_ring,
            second.mesh.first_curve_gap_lower_ring);
  EXPECT_DOUBLE_EQ(first.mesh.first_curve_gap_start_azimuth,
                   second.mesh.first_curve_gap_start_azimuth);
  EXPECT_DOUBLE_EQ(first.mesh.first_curve_gap_end_azimuth,
                   second.mesh.first_curve_gap_end_azimuth);
  EXPECT_EQ(first.mesh.first_curve_gap_corner,
            second.mesh.first_curve_gap_corner);
  expectMeshesExactlyEqual(first.mesh, second.mesh);
}

void expectOutputTopologyValid(const SupportedStripMesh& mesh) {
  IndexedMesh indexed;
  indexed.vertices = mesh.vertices;
  indexed.triangles = mesh.triangles;
  const MeshTopologyValidationResult topology =
      validateMeshTopology(indexed,
                           kOutputTopologyEquivalenceToleranceM);
  EXPECT_TRUE(topology.valid)
      << topology.reason << " degenerate=" << topology.degenerate_triangles
      << " duplicate=" << topology.duplicate_triangles
      << " edge=" << topology.nonmanifold_edges
      << " vertex=" << topology.nonmanifold_vertices;
}

TEST(ScanStripSupportTest,
     OutputTopologyRepairFallsBackForAChainedBowTie) {
  SupportedStripMesh source_mesh;
  source_mesh.vertices = {
      Eigen::Vector3f(20.0f, 0.0f, 0.0f),
      Eigen::Vector3f(21.0f, -1.0f, 0.0f),
      Eigen::Vector3f(21.0f, 1.0f, 0.0f),
      Eigen::Vector3f(20.00004f, 0.0f, 0.0f),
      Eigen::Vector3f(19.0f, -1.0f, 0.0f),
      Eigen::Vector3f(19.0f, 1.0f, 0.0f),
      Eigen::Vector3f(10.0f, 0.0f, 0.0f),
      Eigen::Vector3f(10.0f, -1.0f, 0.0f),
      Eigen::Vector3f(11.0f, -1.0f, 0.0f),
      Eigen::Vector3f(11.0f, 1.0f, 0.0f),
      Eigen::Vector3f(10.0f, 1.0f, 0.0f),
  };

  const MeshColumnKey column_d{0U, 0U, 1U};
  const MeshColumnKey column_c{0U, 1U, 2U};
  const MeshColumnKey column_a{0U, 2U, 3U};
  const MeshColumnKey column_b{0U, 3U, 4U};
  const RawMeshTriangles raw_triangles = {
      RawMeshTriangle{{0U, 1U, 2U}},
      RawMeshTriangle{{3U, 5U, 4U}},
      RawMeshTriangle{{6U, 7U, 8U}},
      RawMeshTriangle{{6U, 8U, 9U}},
      RawMeshTriangle{{6U, 9U, 10U}},
  };
  const std::vector<MeshColumnKey> raw_triangle_columns = {
      column_c, column_d, column_a, column_c, column_b,
  };
  const std::vector<std::uint8_t> vertex_sensor_ids(
      source_mesh.vertices.size(), 0U);

  SensorCloud sensor;
  sensor.sensor_id = 0U;
  sensor.origin = Eigen::Vector3f::Zero();
  const SensorClouds sensors{sensor};
  MeshQuadPlans plans;
  for (std::size_t index = 0U; index < 4U; ++index) {
    MeshQuadPlan plan;
    plan.sensor_id = 0U;
    plan.start_breakpoint = index;
    plan.end_breakpoint = index + 1U;
    plan.quad.strong = false;
    plan.quad.lower_start = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    plan.quad.lower_end = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    plan.quad.upper_start = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    plan.quad.upper_end = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    plans.push_back(plan);
  }

  std::vector<std::size_t> finalized_triangle_counts;
  std::vector<MeshFinalizeStatus> finalized_statuses;
  const auto finalize_mesh =
      [&finalized_triangle_counts, &finalized_statuses](
          const SupportedStripMesh& source,
          const RawMeshTriangles& active_triangles,
          const std::vector<MeshColumnKey>&,
          SupportedStripMesh& finalized_mesh, MeshColumnSet& exact_conflicts,
          std::string& reason) {
        exact_conflicts.clear();
        reason.clear();
        finalized_mesh = source;
        finalized_mesh.triangles.clear();
        finalized_mesh.triangles.reserve(active_triangles.size());
        for (const RawMeshTriangle& triangle : active_triangles) {
          finalized_mesh.triangles.emplace_back(
              static_cast<int>(triangle[0]), static_cast<int>(triangle[1]),
              static_cast<int>(triangle[2]));
        }
        finalized_triangle_counts.push_back(active_triangles.size());

        IndexedMesh indexed;
        indexed.vertices = finalized_mesh.vertices;
        indexed.triangles = finalized_mesh.triangles;
        const MeshTopologyValidationResult exact =
            validateMeshTopology(indexed, 0.0);
        if (!exact.valid) {
          reason = exact.reason;
          finalized_statuses.push_back(
              MeshFinalizeStatus::kExactTopologyInvalid);
          return MeshFinalizeStatus::kExactTopologyInvalid;
        }
        const MeshTopologyValidationResult output = validateMeshTopology(
            indexed, kOutputTopologyEquivalenceToleranceM);
        if (!output.valid) {
          reason = output.reason;
          finalized_statuses.push_back(
              MeshFinalizeStatus::kOutputTopologyInvalid);
          return MeshFinalizeStatus::kOutputTopologyInvalid;
        }
        finalized_statuses.push_back(MeshFinalizeStatus::kAccepted);
        return MeshFinalizeStatus::kAccepted;
      };

  MeshColumnSet filtered_columns;
  RawMeshTriangles active_triangles;
  std::vector<MeshColumnKey> active_columns;
  SupportedStripMesh finalized_mesh;
  bool output_mesh_finalized = false;
  std::string failure_reason;
  std::vector<MeshColumnSet> repair_trace;
  ASSERT_TRUE(repairOutputTopologyColumns(
      source_mesh, raw_triangles, raw_triangle_columns, vertex_sensor_ids,
      sensors, plans, finalize_mesh, filtered_columns, active_triangles,
      active_columns, finalized_mesh, output_mesh_finalized, failure_reason,
      &repair_trace))
      << failure_reason;

  ASSERT_EQ(repair_trace.size(), 2U);
  EXPECT_EQ(repair_trace[0].size(), 1U);
  EXPECT_EQ(repair_trace[0].count(column_c), 1U);
  EXPECT_EQ(repair_trace[1].size(), 2U);
  EXPECT_EQ(repair_trace[1].count(column_a), 1U);
  EXPECT_EQ(repair_trace[1].count(column_b), 1U);

  ASSERT_EQ(finalized_triangle_counts.size(), 2U);
  EXPECT_EQ(finalized_triangle_counts[0], 3U);
  EXPECT_EQ(finalized_triangle_counts[1], 1U);
  ASSERT_EQ(finalized_statuses.size(), 2U);
  EXPECT_EQ(finalized_statuses[0],
            MeshFinalizeStatus::kExactTopologyInvalid);
  EXPECT_EQ(finalized_statuses[1], MeshFinalizeStatus::kAccepted);

  EXPECT_TRUE(output_mesh_finalized);
  EXPECT_TRUE(failure_reason.empty());
  EXPECT_EQ(filtered_columns.size(), 3U);
  EXPECT_EQ(filtered_columns.count(column_c), 1U);
  EXPECT_EQ(filtered_columns.count(column_a), 1U);
  EXPECT_EQ(filtered_columns.count(column_b), 1U);
  EXPECT_EQ(filtered_columns.count(column_d), 0U);
  ASSERT_EQ(active_triangles.size(), 1U);
  ASSERT_EQ(active_columns.size(), 1U);
  EXPECT_EQ(active_columns[0].start_breakpoint,
            column_d.start_breakpoint);
  EXPECT_EQ(active_triangles[0], raw_triangles[1]);
  ASSERT_EQ(finalized_mesh.triangles.size(), 1U);
  EXPECT_EQ(finalized_mesh.triangles[0].x(), 3);
  EXPECT_EQ(finalized_mesh.triangles[0].y(), 5);
  EXPECT_EQ(finalized_mesh.triangles[0].z(), 4);
  expectOutputTopologyValid(finalized_mesh);
}

TEST(ScanStripSupportTest,
     OutputEdgeIncidentsPreserveInlineAndOverflowInsertionOrder) {
  OutputEdgeIncidents incidents;
  EXPECT_EQ(incidents.size(), 0U);
  incidents.pushBack(17U);
  incidents.pushBack(3U);
  EXPECT_TRUE(incidents.overflow_triangles.empty());
  incidents.pushBack(29U);
  incidents.pushBack(11U);

  ASSERT_EQ(incidents.size(), 4U);
  EXPECT_EQ(incidents.inline_triangles[0], 17U);
  EXPECT_EQ(incidents.inline_triangles[1], 3U);
  ASSERT_EQ(incidents.overflow_triangles.size(), 2U);
  EXPECT_EQ(incidents.overflow_triangles[0], 29U);
  EXPECT_EQ(incidents.overflow_triangles[1], 11U);
  const std::array<std::size_t, 4> expected{{17U, 3U, 29U, 11U}};
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    EXPECT_EQ(incidents[index], expected[index]);
  }
}

TEST(ScanStripSupportTest, SourceTopologyRequiresBothGenuineRawFields) {
  TsdfPoint point;
  EXPECT_TRUE(assignSourceTopologyFields(true, 7.0, true, -0.25, point));
  EXPECT_TRUE(point.source_topology_valid);
  EXPECT_EQ(point.ring, 7U);
  EXPECT_FLOAT_EQ(point.azimuth, -0.25f);

  EXPECT_FALSE(assignSourceTopologyFields(true, 7.5, true, 0.25, point));
  EXPECT_FALSE(point.source_topology_valid);
  EXPECT_EQ(point.ring, 0U);
  EXPECT_FLOAT_EQ(point.azimuth, 0.25f);

  EXPECT_FALSE(assignSourceTopologyFields(true, 7.0, false, 0.25, point));
  EXPECT_FALSE(point.source_topology_valid);
  EXPECT_EQ(point.ring, 7U);
  EXPECT_FALSE(std::isfinite(point.azimuth));
  point.azimuth = 0.5f;  // Position-derived fallback stays explicitly invalid.
  EXPECT_FALSE(point.source_topology_valid);

  EXPECT_FALSE(assignSourceTopologyFields(
      true, 7.0, true, std::numeric_limits<double>::infinity(), point));
  EXPECT_FALSE(point.source_topology_valid);
  EXPECT_FALSE(std::isfinite(point.azimuth));

  EXPECT_FALSE(assignSourceTopologyFields(true, 7.0, true, 7.0, point));
  EXPECT_FALSE(point.source_topology_valid);
}

TEST(ScanStripSupportTest, AcceptsContinuousStrongWallStrip) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
                                   {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "accepted");
  EXPECT_GT(result.stats.strong_quads, 0U);
  EXPECT_GE(result.stats.accepted_run_quads, 2U);
  EXPECT_FALSE(result.rays.empty());
  for (const SupportedSurfaceRay& ray : result.rays) {
    EXPECT_FALSE(ray.verified_long_span);
    EXPECT_LE(ray.lower_ring, 1U);
    EXPECT_NEAR(ray.position.x(), 10.0f, 1e-4f);
  }
}

TEST(ScanStripSupportTest, RejectsAnIsolatedSingleQuad) {
  SensorClouds sensors;
  sensors.push_back(
      makeWallSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
                     {-0.1f, 0.1f, 2.0f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.rays.empty());
  EXPECT_GT(result.stats.rejected_isolated_quads, 0U);
}

TEST(ScanStripSupportTest, DoesNotBridgeALargeFieldOfViewGap) {
  SensorClouds sensors;
  sensors.push_back(makeSphereSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {0.0f, 0.2f, 0.4f, 120.0f, 120.2f, 120.4f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.stats.candidate_quads, 10U);
  ASSERT_FALSE(result.rays.empty());
  for (const SupportedSurfaceRay& ray : result.rays) {
    const float degrees = ray.azimuth * 180.0f / kPi;
    const bool first_cluster = degrees >= -0.1f && degrees <= 0.5f;
    const bool second_cluster = degrees >= 119.9f && degrees <= 120.5f;
    EXPECT_TRUE(first_cluster || second_cluster) << degrees;
  }
}

TEST(ScanStripSupportTest, PreservesTheWrapQuadForACompleteCircle) {
  std::vector<float> azimuths;
  for (int index = 0; index < 720; ++index) {
    azimuths.push_back(-180.0f + 0.5f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(
      makeSphereSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.stats.candidate_quads, 2U * azimuths.size());
  EXPECT_EQ(result.stats.accepted_run_quads, 2U * azimuths.size());
  EXPECT_FALSE(result.rays.empty());
}

TEST(ScanStripSupportTest, IndexedPlaneStripHasOnlyOneOuterBoundary) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig config = testConfig();
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty());
  ASSERT_FALSE(result.mesh.triangles.empty());
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectConsistentWindingTowardOrigin(result.mesh, Eigen::Vector3f::Zero());
  EXPECT_EQ(boundaryComponentCount(result.mesh), 1U);

  const std::map<MeshEdge, std::size_t> counts =
      meshEdgeCounts(result.mesh);
  for (const auto& edge : counts) {
    if (edge.second != 1U) {
      continue;
    }
    for (const int vertex_index : {edge.first.first, edge.first.second}) {
      const Eigen::Vector3f& vertex = result.mesh.vertices[vertex_index];
      // The only boundary is the two outer rings (|z| ~= 0.17 m) plus
      // the two end-azimuth sides (|y| ~= 0.07 m).  z=0 away from those
      // sides would expose an internal ring-pair/quad crack.
      EXPECT_TRUE(std::fabs(vertex.z()) > 0.08f ||
                  std::fabs(vertex.y()) > 0.06f)
          << vertex.transpose();
    }
  }
}

TEST(ScanStripSupportTest,
     IndexedDenseSupportedColumnsUseFewerCommonBreakpoints) {
  std::vector<float> azimuths;
  for (int index = -20; index <= 20; ++index) {
    azimuths.push_back(0.1f * static_cast<float>(index));
  }
  SensorCloud ordered = makeWallSensor(
      {0U, 1U, 2U, 3U}, {-1.5f, -0.5f, 0.5f, 1.5f}, azimuths, 10.0f);
  SensorCloud reversed = ordered;
  std::reverse(reversed.points.begin(), reversed.points.end());

  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.9f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(SensorClouds{ordered}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);

  ASSERT_TRUE(ordered_result.accepted) << ordered_result.reason;
  ASSERT_TRUE(ordered_result.mesh.accepted) << ordered_result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_GT(ordered_result.stats.accepted_run_quads, 0U);
  ASSERT_GT(ordered_result.mesh.curve_intervals, 0U);
  EXPECT_LT(ordered_result.mesh.curve_intervals,
            ordered_result.stats.accepted_run_quads / 2U);
  expectSupportResultsExactlyEqual(ordered_result, reversed_result);
  expectBoundedManifoldEdges(
      ordered_result.mesh,
      config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(ordered_result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(ordered_result.mesh), 0U);
  EXPECT_EQ(boundaryComponentCount(ordered_result.mesh), 1U);
}

TEST(ScanStripSupportTest,
     IndexedDenseCoarseningHonorsExactFinalMeshBudget) {
  std::vector<float> azimuths;
  for (int index = -400; index <= 400; ++index) {
    azimuths.push_back(0.005f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U, 3U}, {-1.5f, -0.5f, 0.5f, 1.5f}, azimuths, 10.0f));

  ScanStripSupportConfig reference_config = testConfig();
  reference_config.build_surface_rays = false;
  reference_config.build_indexed_mesh = true;
  reference_config.maximum_mesh_edge_voxels = 2.9f;
  reference_config.max_mesh_vertices = 100000U;
  reference_config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult reference =
      buildSupportedScanStrips(sensors, reference_config);
  ASSERT_TRUE(reference.accepted) << reference.reason;
  ASSERT_TRUE(reference.mesh.accepted) << reference.mesh.reason;
  ASSERT_GT(reference.mesh.vertices.size(), 1U);
  ASSERT_GT(reference.mesh.triangles.size(), 1U);
  ASSERT_GT(reference.stats.accepted_run_quads,
            reference.mesh.triangles.size());

  ScanStripSupportConfig exact = reference_config;
  exact.max_mesh_vertices = reference.mesh.vertices.size();
  exact.max_mesh_triangles = reference.mesh.triangles.size();
  const ScanStripSupportResult exact_result =
      buildSupportedScanStrips(sensors, exact);
  ASSERT_TRUE(exact_result.accepted) << exact_result.reason;
  ASSERT_TRUE(exact_result.mesh.accepted) << exact_result.mesh.reason;
  expectSupportResultsExactlyEqual(reference, exact_result);

  ScanStripSupportConfig vertex_limited = exact;
  --vertex_limited.max_mesh_vertices;
  const ScanStripSupportResult vertex_failure =
      buildSupportedScanStrips(sensors, vertex_limited);
  ASSERT_TRUE(vertex_failure.accepted) << vertex_failure.reason;
  EXPECT_FALSE(vertex_failure.mesh.accepted);
  EXPECT_TRUE(vertex_failure.mesh.budget_limited);
  EXPECT_EQ(vertex_failure.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(vertex_failure.mesh.vertices.empty());
  EXPECT_TRUE(vertex_failure.mesh.triangles.empty());

  ScanStripSupportConfig triangle_limited = exact;
  --triangle_limited.max_mesh_triangles;
  const ScanStripSupportResult triangle_failure =
      buildSupportedScanStrips(sensors, triangle_limited);
  ASSERT_TRUE(triangle_failure.accepted) << triangle_failure.reason;
  EXPECT_FALSE(triangle_failure.mesh.accepted);
  EXPECT_TRUE(triangle_failure.mesh.budget_limited);
  EXPECT_EQ(triangle_failure.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(triangle_failure.mesh.vertices.empty());
  EXPECT_TRUE(triangle_failure.mesh.triangles.empty());
}

TEST(ScanStripSupportTest,
     IndexedCommonBreakpointsNeverCrossAnUnsupportedGap) {
  SensorClouds sensors;
  sensors.push_back(makeSphereSensor(
      {0U, 1U, 2U, 3U}, {-1.5f, -0.5f, 0.5f, 1.5f},
      {-1.2f, -1.0f, -0.8f, -0.6f, 3.0f, 3.2f, 3.4f, 3.6f}, 10.0f));
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.9f;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);

  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_FALSE(result.mesh.triangles.empty());
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
  EXPECT_EQ(boundaryComponentCount(result.mesh), 2U);
  for (const Eigen::Vector3i& triangle : result.mesh.triangles) {
    const Eigen::Vector3f center =
        (result.mesh.vertices[triangle.x()] +
         result.mesh.vertices[triangle.y()] +
         result.mesh.vertices[triangle.z()]) /
        3.0f;
    const float degrees = std::atan2(center.y(), center.x()) * 180.0f / kPi;
    EXPECT_TRUE((degrees >= -1.3f && degrees <= -0.5f) ||
                (degrees >= 2.9f && degrees <= 3.7f))
        << degrees;
  }
}

TEST(ScanStripSupportTest,
     IndexedLargeEdgeLimitStartsWithTwoTrianglesPerMicroquad) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 10.0f;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_GT(result.mesh.curve_intervals, 0U);
  EXPECT_EQ(result.mesh.triangles.size(), 2U * result.mesh.curve_intervals);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
}

TEST(ScanStripSupportTest,
     IndexedCrossEdgeZipperIsConformingAndActuallySplits) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_GT(result.mesh.curve_intervals, 0U);
  EXPECT_GT(result.mesh.triangles.size(), 2U * result.mesh.curve_intervals);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
  EXPECT_EQ(boundaryComponentCount(result.mesh), 1U);
}

TEST(ScanStripSupportTest,
     IndexedCrossEdgeZipperBudgetsFailLateAndAtomically) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig reference_config = testConfig();
  reference_config.build_surface_rays = false;
  reference_config.build_indexed_mesh = true;
  const ScanStripSupportResult reference =
      buildSupportedScanStrips(sensors, reference_config);
  ASSERT_TRUE(reference.mesh.accepted) << reference.mesh.reason;
  ASSERT_GT(reference.mesh.triangles.size(),
            2U * reference.mesh.curve_intervals);
  ASSERT_GT(reference.mesh.vertices.size(), 1U);
  ASSERT_GT(reference.mesh.triangles.size(), 1U);

  ScanStripSupportConfig vertex_limited = reference_config;
  vertex_limited.max_mesh_vertices = reference.mesh.vertices.size() - 1U;
  const ScanStripSupportResult vertex_failure =
      buildSupportedScanStrips(sensors, vertex_limited);
  ASSERT_TRUE(vertex_failure.accepted) << vertex_failure.reason;
  EXPECT_FALSE(vertex_failure.mesh.accepted);
  EXPECT_TRUE(vertex_failure.mesh.budget_limited);
  EXPECT_EQ(vertex_failure.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(vertex_failure.mesh.vertices.empty());
  EXPECT_TRUE(vertex_failure.mesh.triangles.empty());

  ScanStripSupportConfig triangle_limited = reference_config;
  triangle_limited.max_mesh_triangles =
      reference.mesh.triangles.size() - 1U;
  const ScanStripSupportResult triangle_failure =
      buildSupportedScanStrips(sensors, triangle_limited);
  ASSERT_TRUE(triangle_failure.accepted) << triangle_failure.reason;
  EXPECT_FALSE(triangle_failure.mesh.accepted);
  EXPECT_TRUE(triangle_failure.mesh.budget_limited);
  EXPECT_EQ(triangle_failure.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(triangle_failure.mesh.vertices.empty());
  EXPECT_TRUE(triangle_failure.mesh.triangles.empty());

  ScanStripSupportConfig exact = reference_config;
  exact.max_mesh_vertices = reference.mesh.vertices.size();
  exact.max_mesh_triangles = reference.mesh.triangles.size();
  const ScanStripSupportResult exact_result =
      buildSupportedScanStrips(sensors, exact);
  ASSERT_TRUE(exact_result.mesh.accepted) << exact_result.mesh.reason;
  expectSupportResultsExactlyEqual(reference, exact_result);
}

TEST(ScanStripSupportTest,
     IndexedUnequalCrossLengthsRefineLocallyInsteadOfWholeRun) {
  std::vector<float> azimuths;
  for (int index = 0; index <= 700; ++index) {
    // Keep each global-breakpoint interval below the edge limit even near
    // 70 degrees, while cross-ring spans still grow substantially with range.
    azimuths.push_back(0.1f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths, 10.0f));
  ScanStripSupportConfig config = testConfig();
  config.max_range = 35.0f;
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.0f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_FALSE(result.mesh.triangles.empty());

  std::size_t near_triangles = 0U;
  std::size_t far_triangles = 0U;
  for (const Eigen::Vector3i& triangle : result.mesh.triangles) {
    const Eigen::Vector3f centroid =
        (result.mesh.vertices[triangle.x()] +
         result.mesh.vertices[triangle.y()] +
         result.mesh.vertices[triangle.z()]) /
        3.0f;
    const float azimuth_degrees =
        std::atan2(centroid.y(), centroid.x()) * 180.0f / kPi;
    if (azimuth_degrees >= 0.0f && azimuth_degrees < 10.0f) {
      ++near_triangles;
    } else if (azimuth_degrees > 60.0f && azimuth_degrees <= 70.0f) {
      ++far_triangles;
    }
  }
  ASSERT_GT(near_triangles, 0U);
  ASSERT_GT(far_triangles, 0U);
  EXPECT_GT(far_triangles, 2U * near_triangles);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
}

TEST(ScanStripSupportTest,
     IndexedZipperSubdividesAnOverlongAzimuthIntervalSensorWide) {
  std::vector<float> azimuths;
  for (int index = 0; index <= 140; ++index) {
    azimuths.push_back(0.5f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths, 10.0f));
  ScanStripSupportConfig config = testConfig();
  config.max_range = 35.0f;
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.0f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty());
  ASSERT_FALSE(result.mesh.triangles.empty());
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);

  ScanStripSupportConfig exact = config;
  exact.max_mesh_vertices = result.mesh.vertices.size();
  exact.max_mesh_triangles = result.mesh.triangles.size();
  const ScanStripSupportResult exact_result =
      buildSupportedScanStrips(sensors, exact);
  ASSERT_TRUE(exact_result.mesh.accepted) << exact_result.mesh.reason;
  expectMeshesExactlyEqual(result.mesh, exact_result.mesh);

  ASSERT_GT(result.mesh.vertices.size(), 1U);
  ScanStripSupportConfig vertex_limited = exact;
  vertex_limited.max_mesh_vertices = result.mesh.vertices.size() - 1U;
  const ScanStripSupportResult vertex_failure =
      buildSupportedScanStrips(sensors, vertex_limited);
  EXPECT_FALSE(vertex_failure.mesh.accepted);
  EXPECT_TRUE(vertex_failure.mesh.budget_limited);
  EXPECT_TRUE(vertex_failure.mesh.vertices.empty());
  EXPECT_TRUE(vertex_failure.mesh.triangles.empty());

  ASSERT_GT(result.mesh.triangles.size(), 1U);
  ScanStripSupportConfig triangle_limited = exact;
  triangle_limited.max_mesh_triangles = result.mesh.triangles.size() - 1U;
  const ScanStripSupportResult triangle_failure =
      buildSupportedScanStrips(sensors, triangle_limited);
  EXPECT_FALSE(triangle_failure.mesh.accepted);
  EXPECT_TRUE(triangle_failure.mesh.budget_limited);
  EXPECT_TRUE(triangle_failure.mesh.vertices.empty());
  EXPECT_TRUE(triangle_failure.mesh.triangles.empty());
}

TEST(ScanStripSupportTest,
     IndexedAsynchronousRingPairDomainsMaskTheTouchingSensorColumns) {
  const std::vector<float> lower = {0.0f, 0.2f, 0.4f, 0.6f};
  const std::vector<float> shared =
      {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f};
  const std::vector<float> upper = {0.6f, 0.8f, 1.0f, 1.2f};
  SensorCloud ordered = makeUnevenSphereSensor(
      {lower, shared, upper}, {-1.0f, 0.0f, 1.0f}, 10.0f);
  SensorCloud reversed = ordered;
  std::reverse(reversed.points.begin(), reversed.points.end());
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.9f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 100000U;

  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(SensorClouds{ordered}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);
  ASSERT_TRUE(ordered_result.accepted) << ordered_result.reason;
  ASSERT_TRUE(reversed_result.accepted) << reversed_result.reason;
  ASSERT_TRUE(ordered_result.mesh.accepted) << ordered_result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_FALSE(ordered_result.mesh.triangles.empty())
      << ordered_result.mesh.reason << " skipped="
      << ordered_result.mesh.skipped_sensor_columns << " intervals="
      << ordered_result.mesh.curve_intervals;
  EXPECT_GE(ordered_result.mesh.skipped_sensor_columns, 2U);
  expectSupportResultsExactlyEqual(ordered_result, reversed_result);
  expectBoundedManifoldEdges(
      ordered_result.mesh,
      config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(ordered_result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(ordered_result.mesh), 0U);
}

TEST(ScanStripSupportTest, IndexedFullCircleHasNoAzimuthSeam) {
  std::vector<float> azimuths;
  for (int index = 0; index < 720; ++index) {
    azimuths.push_back(-180.0f + 0.5f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(
      makeSphereSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths));
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_GT(result.mesh.triangles.size(),
            2U * result.mesh.curve_intervals);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
  expectConsistentWindingTowardOrigin(result.mesh, Eigen::Vector3f::Zero());
  // A closed strip is topologically a cylinder: only the two outer-ring
  // boundary loops remain.  Cutting last->first would collapse this to one
  // rectangular loop containing two extra seam edges.
  EXPECT_EQ(boundaryComponentCount(result.mesh), 2U);
  for (const auto& edge : meshEdgeCounts(result.mesh)) {
    if (edge.second != 1U) {
      continue;
    }
    EXPECT_GT(std::fabs(result.mesh.vertices[edge.first.first].z()), 0.08f);
    EXPECT_GT(std::fabs(result.mesh.vertices[edge.first.second].z()), 0.08f);
  }
}

TEST(ScanStripSupportTest, IndexedUnevenRingSamplingHasNoInternalBoundary) {
  std::vector<float> coarse;
  std::vector<float> fine;
  for (int index = -10; index <= 10; ++index) {
    coarse.push_back(0.4f * static_cast<float>(index));
  }
  for (int index = -20; index <= 20; ++index) {
    fine.push_back(0.2f * static_cast<float>(index));
  }
  SensorClouds sensors;
  sensors.push_back(makeUnevenSphereSensor(
      {coarse, fine, fine}, {-1.0f, 0.0f, 1.0f}, 30.0f));
  ScanStripSupportConfig config = testConfig();
  config.build_indexed_mesh = true;
  config.max_range = 31.0f;
  config.max_mesh_vertices = 20000U;
  config.max_mesh_triangles = 40000U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty());
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectConsistentWindingTowardOrigin(result.mesh, Eigen::Vector3f::Zero());
  EXPECT_EQ(boundaryComponentCount(result.mesh), 1U);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);

  std::size_t internal_boundary_edges = 0U;
  for (const auto& edge : meshEdgeCounts(result.mesh)) {
    if (edge.second != 1U) {
      continue;
    }
    const Eigen::Vector3f& first = result.mesh.vertices[edge.first.first];
    const Eigen::Vector3f& second = result.mesh.vertices[edge.first.second];
    const auto is_outer = [](const Eigen::Vector3f& vertex) {
      return std::fabs(vertex.z()) > 0.25f ||
             std::fabs(vertex.y()) > 2.0f;
    };
    if (!is_outer(first) || !is_outer(second)) {
      ++internal_boundary_edges;
    }
  }
  EXPECT_EQ(internal_boundary_edges, 0U);
}

TEST(ScanStripSupportTest,
     IndexedPhaseOffsetMissingColumnsUseOneSensorWideMask) {
  const std::vector<float> base =
      {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f};
  const std::vector<float> shifted =
      {0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.3f};
  SensorCloud sensor = makeUnevenSphereSensor(
      {base, shifted, base, shifted}, {-1.5f, -0.5f, 0.5f, 1.5f},
      10.0f);
  SensorCloud reversed = sensor;
  std::reverse(reversed.points.begin(), reversed.points.end());
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(SensorClouds{sensor}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty()) << result.mesh.reason;
  EXPECT_GT(result.mesh.skipped_curve_intervals, 0U);
  EXPECT_GT(result.mesh.skipped_sensor_columns, 0U);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_GT(boundaryComponentCount(result.mesh), 0U);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
  expectNoAzimuthEdgeWiderThan(result.mesh, 0.5f);
  expectSupportResultsExactlyEqual(result, reversed_result);
}

TEST(ScanStripSupportTest,
     IndexedWrapPhaseOffsetMissingColumnsUseOneSensorWideMask) {
  const std::vector<float> base =
      {359.4f, 359.6f, 359.8f, 0.0f, 0.2f, 0.4f, 0.6f};
  const std::vector<float> shifted =
      {359.5f, 359.7f, 359.9f, 0.1f, 0.3f, 0.5f, 0.7f};
  SensorCloud sensor = makeUnevenSphereSensor(
      {base, shifted, base, shifted}, {-1.5f, -0.5f, 0.5f, 1.5f},
      10.0f);
  SensorCloud reversed = sensor;
  std::reverse(reversed.points.begin(), reversed.points.end());
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(SensorClouds{sensor}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty()) << result.mesh.reason;
  EXPECT_GT(result.mesh.skipped_curve_intervals, 0U);
  EXPECT_GT(result.mesh.skipped_sensor_columns, 0U);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_GT(boundaryComponentCount(result.mesh), 0U);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U);
  expectNoAzimuthEdgeWiderThan(result.mesh, 0.5f);
  expectSupportResultsExactlyEqual(result, reversed_result);
}

TEST(ScanStripSupportTest,
     IndexedNearCoincidentPhaseBreakpointsUseSemanticTopologyKeys) {
  const std::vector<float> base =
      {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f};
  std::vector<float> shifted = base;
  for (float& azimuth : shifted) {
    // Just above the global-breakpoint merge threshold at max_range=30 m,
    // and about one configured weld tolerance apart at 20 m. These columns
    // must remain distinct semantic topology even though their XYZ keys are
    // nearly coincident.
    azimuth += 0.000045f;
  }
  SensorCloud ordered = makeUnevenSphereSensor(
      {base, shifted, base, shifted}, {-1.5f, -0.5f, 0.5f, 1.5f}, 20.0f);
  ordered.sensor_id = 7U;
  SensorCloud reversed = ordered;
  std::reverse(reversed.points.begin(), reversed.points.end());

  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.0f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(SensorClouds{ordered}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);
  ASSERT_TRUE(ordered_result.accepted) << ordered_result.reason;
  ASSERT_TRUE(reversed_result.accepted) << reversed_result.reason;
  ASSERT_TRUE(ordered_result.mesh.accepted) << ordered_result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_FALSE(ordered_result.mesh.triangles.empty());
  EXPECT_GT(ordered_result.mesh.skipped_sensor_columns, 0U);
  expectSupportResultsExactlyEqual(ordered_result, reversed_result);
  expectOutputTopologyValid(ordered_result.mesh);
  expectBoundedManifoldEdges(
      ordered_result.mesh,
      config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(ordered_result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(ordered_result.mesh), 0U);

  ScanStripSupportConfig exact_budget = config;
  exact_budget.max_mesh_vertices = ordered_result.mesh.vertices.size();
  exact_budget.max_mesh_triangles = ordered_result.mesh.triangles.size();
  const ScanStripSupportResult exact_result =
      buildSupportedScanStrips(SensorClouds{ordered}, exact_budget);
  ASSERT_TRUE(exact_result.mesh.accepted) << exact_result.mesh.reason;
  expectSupportResultsExactlyEqual(ordered_result, exact_result);

  ASSERT_GT(ordered_result.mesh.vertices.size(), 1U);
  ScanStripSupportConfig vertex_limited = exact_budget;
  vertex_limited.max_mesh_vertices = ordered_result.mesh.vertices.size() - 1U;
  const ScanStripSupportResult vertex_failure =
      buildSupportedScanStrips(SensorClouds{ordered}, vertex_limited);
  EXPECT_FALSE(vertex_failure.mesh.accepted);
  EXPECT_TRUE(vertex_failure.mesh.budget_limited);
  EXPECT_TRUE(vertex_failure.mesh.vertices.empty());
  EXPECT_TRUE(vertex_failure.mesh.triangles.empty());

  ASSERT_GT(ordered_result.mesh.triangles.size(), 1U);
  ScanStripSupportConfig triangle_limited = exact_budget;
  triangle_limited.max_mesh_triangles =
      ordered_result.mesh.triangles.size() - 1U;
  const ScanStripSupportResult triangle_failure =
      buildSupportedScanStrips(SensorClouds{ordered}, triangle_limited);
  EXPECT_FALSE(triangle_failure.mesh.accepted);
  EXPECT_TRUE(triangle_failure.mesh.budget_limited);
  EXPECT_TRUE(triangle_failure.mesh.vertices.empty());
  EXPECT_TRUE(triangle_failure.mesh.triangles.empty());
}

TEST(ScanStripSupportTest,
     IndexedNearMergedBreakpointWithSpatialMismatchRemainsACut) {
  const std::vector<std::uint16_t> rings = {0U, 1U, 2U, 3U};
  const std::vector<float> elevations = {-1.5f, -0.5f, 0.5f, 1.5f};
  const std::vector<std::pair<float, float>> azimuth_range = {
      {-0.4f, 10.0f}, {-0.2f, 10.0f}, {0.0f, 10.0f},
      {0.00002f, 12.0f}, {0.2f, 12.0f}, {0.4f, 12.0f}};
  SensorCloud ordered;
  ordered.sensor_id = 5U;
  for (std::size_t ring_index = 0U; ring_index < rings.size();
       ++ring_index) {
    for (const auto& sample : azimuth_range) {
      const float azimuth = radians(sample.first);
      ordered.points.push_back(topologyPoint(
          rayDirection(azimuth, radians(elevations[ring_index])) *
              sample.second,
          rings[ring_index], azimuth));
    }
  }
  SensorCloud reversed = ordered;
  std::reverse(reversed.points.begin(), reversed.points.end());

  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.9f;
  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(SensorClouds{ordered}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);

  ASSERT_TRUE(ordered_result.accepted) << ordered_result.reason;
  ASSERT_TRUE(ordered_result.mesh.accepted) << ordered_result.mesh.reason;
  ASSERT_FALSE(ordered_result.mesh.triangles.empty());
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  expectSupportResultsExactlyEqual(ordered_result, reversed_result);
  expectBoundedManifoldEdges(
      ordered_result.mesh,
      config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(ordered_result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(ordered_result.mesh), 0U);
  EXPECT_EQ(boundaryComponentCount(ordered_result.mesh), 2U);
  for (const Eigen::Vector3i& triangle : ordered_result.mesh.triangles) {
    const Eigen::Vector3f center =
        (ordered_result.mesh.vertices[triangle.x()] +
         ordered_result.mesh.vertices[triangle.y()] +
         ordered_result.mesh.vertices[triangle.z()]) /
        3.0f;
    EXPECT_TRUE(center.norm() < 10.5f || center.norm() > 11.5f)
        << center.transpose();
  }
}

TEST(ScanStripSupportTest,
     IndexedNearCoincidentColumnsDoNotCreateABowTie) {
  const std::vector<float> base =
      {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.2f};
  std::vector<float> shifted = base;
  for (float& azimuth : shifted) {
    azimuth += 0.00007f;
  }
  SensorCloud sensor = makeUnevenSphereSensor(
      {base, shifted, base, shifted}, {-1.5f, -0.5f, 0.5f, 1.5f}, 20.0f);
  SensorCloud reversed = sensor;
  std::reverse(reversed.points.begin(), reversed.points.end());
  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  config.maximum_mesh_edge_voxels = 2.0f;
  config.max_mesh_vertices = 100000U;
  config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(SensorClouds{sensor}, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(SensorClouds{reversed}, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.mesh.accepted) << result.mesh.reason;
  ASSERT_TRUE(reversed_result.mesh.accepted) << reversed_result.mesh.reason;
  ASSERT_FALSE(result.mesh.vertices.empty());
  ASSERT_FALSE(result.mesh.triangles.empty());
  EXPECT_GT(result.mesh.skipped_sensor_columns, 0U);
  EXPECT_LE(result.mesh.skipped_sensor_columns, 2U * base.size());
  EXPECT_GT(result.mesh.output_equivalence_input_triangles,
            result.mesh.triangles.size());
  EXPECT_EQ(result.mesh.output_equivalence_removed_triangles,
            result.mesh.output_equivalence_input_triangles -
                result.mesh.triangles.size());
  EXPECT_GT(result.mesh.output_equivalence_masked_sensor_columns, 0U);
  EXPECT_LE(result.mesh.output_equivalence_masked_sensor_columns,
            result.mesh.skipped_sensor_columns);
  expectSupportResultsExactlyEqual(result, reversed_result);
  expectOutputTopologyValid(result.mesh);
  expectBoundedManifoldEdges(
      result.mesh, config.voxel_size * config.maximum_mesh_edge_voxels);
  expectVertexLinksManifold(result.mesh);
  EXPECT_EQ(boundaryEdgesContainingAnotherVertex(result.mesh), 0U)
      << result.mesh.reason << " skipped="
      << result.mesh.skipped_sensor_columns << " intervals="
      << result.mesh.curve_intervals;
}

TEST(ScanStripSupportTest, IndexedMeshIsStableUnderSensorAndPointPermutation) {
  const std::vector<float> azimuths =
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  SensorCloud first = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths, 10.0f);
  first.sensor_id = 3U;
  SensorCloud second = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths, 12.0f);
  second.sensor_id = 9U;
  SensorClouds ordered;
  ordered.push_back(first);
  ordered.push_back(second);
  SensorClouds permuted;
  std::reverse(second.points.begin(), second.points.end());
  std::reverse(first.points.begin(), first.points.end());
  permuted.push_back(second);
  permuted.push_back(first);

  ScanStripSupportConfig config = testConfig();
  config.build_surface_rays = false;
  config.build_indexed_mesh = true;
  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(ordered, config);
  const ScanStripSupportResult permuted_result =
      buildSupportedScanStrips(permuted, config);
  ASSERT_TRUE(ordered_result.mesh.accepted) << ordered_result.mesh.reason;
  ASSERT_TRUE(permuted_result.mesh.accepted) << permuted_result.mesh.reason;
  ASSERT_GT(ordered_result.mesh.triangles.size(),
            2U * ordered_result.mesh.curve_intervals);
  expectSupportResultsExactlyEqual(ordered_result, permuted_result);
}

TEST(ScanStripSupportTest, IndexedMeshRejectsDepthCurtain) {
  SensorCloud sensor;
  sensor.sensor_id = 0U;
  const std::vector<float> azimuths = {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  for (const float azimuth_deg : azimuths) {
    const float azimuth = radians(azimuth_deg);
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(-1.0f)) * 5.0f, 0U, azimuth));
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, 0.0f) * 12.0f, 1U, azimuth));
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(1.0f)) * 5.0f, 2U, azimuth));
  }
  SensorClouds sensors;
  sensors.push_back(sensor);
  ScanStripSupportConfig config = testConfig();
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.mesh.accepted) << result.mesh.reason;
  EXPECT_EQ(result.mesh.reason, "no_supported_strips");
  EXPECT_TRUE(result.mesh.vertices.empty());
  EXPECT_TRUE(result.mesh.triangles.empty());
}

TEST(ScanStripSupportTest, IndexedMeshBudgetsFailAtomically) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));

  ScanStripSupportConfig reference_config = testConfig();
  reference_config.build_surface_rays = false;
  reference_config.build_indexed_mesh = true;
  reference_config.max_mesh_vertices = 100000U;
  reference_config.max_mesh_triangles = 200000U;
  const ScanStripSupportResult reference =
      buildSupportedScanStrips(sensors, reference_config);
  ASSERT_TRUE(reference.accepted) << reference.reason;
  ASSERT_TRUE(reference.mesh.accepted) << reference.mesh.reason;
  ASSERT_GT(reference.mesh.vertices.size(), 1U);
  ASSERT_GT(reference.mesh.triangles.size(), 1U);

  ScanStripSupportConfig exact_config = reference_config;
  exact_config.max_mesh_vertices = reference.mesh.vertices.size();
  exact_config.max_mesh_triangles = reference.mesh.triangles.size();
  const ScanStripSupportResult exact_result =
      buildSupportedScanStrips(sensors, exact_config);
  ASSERT_TRUE(exact_result.mesh.accepted) << exact_result.mesh.reason;
  expectSupportResultsExactlyEqual(reference, exact_result);

  ScanStripSupportConfig vertex_config = exact_config;
  --vertex_config.max_mesh_vertices;
  const ScanStripSupportResult vertex_result =
      buildSupportedScanStrips(sensors, vertex_config);
  ASSERT_TRUE(vertex_result.accepted) << vertex_result.reason;
  EXPECT_FALSE(vertex_result.mesh.accepted);
  EXPECT_TRUE(vertex_result.mesh.budget_limited);
  EXPECT_EQ(vertex_result.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(vertex_result.mesh.vertices.empty());
  EXPECT_TRUE(vertex_result.mesh.triangles.empty());

  ScanStripSupportConfig triangle_config = exact_config;
  --triangle_config.max_mesh_triangles;
  const ScanStripSupportResult triangle_result =
      buildSupportedScanStrips(sensors, triangle_config);
  ASSERT_TRUE(triangle_result.accepted) << triangle_result.reason;
  EXPECT_FALSE(triangle_result.mesh.accepted);
  EXPECT_TRUE(triangle_result.mesh.budget_limited);
  EXPECT_EQ(triangle_result.mesh.reason, "mesh_budget_limited");
  EXPECT_TRUE(triangle_result.mesh.vertices.empty());
  EXPECT_TRUE(triangle_result.mesh.triangles.empty());
}

TEST(ScanStripSupportTest, IndexedMeshFailsClosedOnInvalidAndOverflowScale) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));

  ScanStripSupportConfig nonfinite = testConfig();
  nonfinite.build_indexed_mesh = true;
  nonfinite.maximum_mesh_edge_voxels =
      std::numeric_limits<float>::infinity();
  const ScanStripSupportResult nonfinite_result =
      buildSupportedScanStrips(sensors, nonfinite);
  EXPECT_FALSE(nonfinite_result.accepted);
  EXPECT_EQ(nonfinite_result.reason, "invalid_configuration");
  EXPECT_TRUE(nonfinite_result.mesh.vertices.empty());
  EXPECT_TRUE(nonfinite_result.mesh.triangles.empty());

  ScanStripSupportConfig overflow = testConfig();
  overflow.build_indexed_mesh = true;
  overflow.mesh_weld_tolerance_voxels =
      std::numeric_limits<float>::denorm_min();
  const ScanStripSupportResult overflow_result =
      buildSupportedScanStrips(sensors, overflow);
  ASSERT_TRUE(overflow_result.accepted) << overflow_result.reason;
  EXPECT_FALSE(overflow_result.mesh.accepted);
  EXPECT_EQ(overflow_result.mesh.reason, "mesh_coordinate_overflow");
  EXPECT_TRUE(overflow_result.mesh.vertices.empty());
  EXPECT_TRUE(overflow_result.mesh.triangles.empty());
}

TEST(ScanStripSupportTest, IndexedMeshRejectsWeldDegeneracy) {
  SensorClouds sensors;
  sensors.push_back(makeSphereSensor(
      {0U, 1U, 2U}, {-0.3f, 0.0f, 0.3f}, {-0.01f, 0.0f, 0.01f}, 2.0f));
  ScanStripSupportConfig config = testConfig();
  config.build_indexed_mesh = true;
  config.mesh_weld_tolerance_voxels = 0.01f;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.mesh.accepted);
  EXPECT_EQ(result.mesh.reason, "no_nondegenerate_strips");
  EXPECT_GT(result.mesh.skipped_degenerate_intervals, 0U);
  EXPECT_GT(result.mesh.skipped_sensor_columns, 0U);
  EXPECT_TRUE(result.mesh.vertices.empty());
  EXPECT_TRUE(result.mesh.triangles.empty());
}

TEST(ScanStripSupportTest, IndexedMeshRejectsDuplicateSensorIds) {
  SensorCloud first = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}, 10.0f);
  SensorCloud second = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}, 12.0f);
  first.sensor_id = 4U;
  second.sensor_id = 4U;
  SensorClouds sensors;
  sensors.push_back(first);
  sensors.push_back(second);
  ScanStripSupportConfig config = testConfig();
  config.build_indexed_mesh = true;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_FALSE(result.mesh.accepted);
  EXPECT_EQ(result.mesh.reason, "duplicate_sensor_id");
  EXPECT_TRUE(result.mesh.vertices.empty());
  EXPECT_TRUE(result.mesh.triangles.empty());
}

TEST(ScanStripSupportTest, FailsClosedWithOnlyTwoObservableRings) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor({0U, 1U}, {-1.0f, 1.0f},
                                   {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_GT(result.stats.strong_quads, 0U);
  EXPECT_EQ(result.stats.rejected_ring_order_pairs, 1U);
  EXPECT_TRUE(result.rays.empty());
}

TEST(ScanStripSupportTest, NeverConnectsNonAdjacentRingIds) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor({0U, 2U}, {-1.0f, 1.0f},
                                   {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.stats.ring_pairs, 0U);
  EXPECT_TRUE(result.rays.empty());
}

TEST(ScanStripSupportTest, LongSpanRequiresAThirdCoplanarRing) {
  const std::vector<float> azimuths = {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  SensorClouds unsupported;
  unsupported.push_back(
      makeGroundSensor({0U, 1U}, {-10.0f, -12.0f}, azimuths));
  const ScanStripSupportResult rejected =
      buildSupportedScanStrips(unsupported, testConfig());
  ASSERT_TRUE(rejected.accepted) << rejected.reason;
  EXPECT_GT(rejected.stats.rejected_ring_order_pairs, 0U);
  EXPECT_TRUE(rejected.rays.empty());

  SensorClouds supported;
  supported.push_back(makeGroundSensor(
      {0U, 1U, 2U}, {-10.0f, -12.0f, -14.0f}, azimuths));
  const ScanStripSupportResult accepted =
      buildSupportedScanStrips(supported, testConfig());
  ASSERT_TRUE(accepted.accepted) << accepted.reason;
  EXPECT_GT(accepted.stats.verified_long_quads, 0U);
  ASSERT_FALSE(accepted.rays.empty());
  EXPECT_TRUE(std::any_of(
      accepted.rays.begin(), accepted.rays.end(),
      [](const SupportedSurfaceRay& ray) { return ray.verified_long_span; }));
  for (const SupportedSurfaceRay& ray : accepted.rays) {
    EXPECT_NEAR(ray.position.z(), -2.0f, 1e-4f);
  }
}

TEST(ScanStripSupportTest, RejectsCrossWallDepthCurtain) {
  SensorCloud sensor;
  sensor.sensor_id = 0;
  const std::vector<float> azimuths = {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  for (const float azimuth_deg : azimuths) {
    const float azimuth = radians(azimuth_deg);
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(-1.0f)) * 5.0f, 0U, azimuth));
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(1.0f)) * 12.0f, 1U, azimuth));
  }
  SensorClouds sensors;
  sensors.push_back(sensor);
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.rays.empty());
  EXPECT_EQ(result.stats.locally_valid_quads, 0U);
}

TEST(ScanStripSupportTest, ModerateDepthCurtainIsNotStrongOrTwoRingSupported) {
  SensorCloud sensor;
  sensor.sensor_id = 0;
  const std::vector<float> azimuths = {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  for (const float azimuth_deg : azimuths) {
    const float azimuth = radians(azimuth_deg);
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(-1.0f)) * 20.0f, 0U, azimuth));
    sensor.points.push_back(topologyPoint(
        rayDirection(azimuth, radians(1.0f)) * 21.0f, 1U, azimuth));
  }
  SensorClouds sensors;
  sensors.push_back(sensor);
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_GT(result.stats.locally_valid_quads, 0U);
  EXPECT_EQ(result.stats.strong_quads, 0U);
  EXPECT_TRUE(result.rays.empty());
}

TEST(ScanStripSupportTest, RejectsNumericallyAdjacentScrambledRingOrder) {
  SensorClouds sensors;
  sensors.push_back(makeSphereSensor(
      {0U, 1U, 2U}, {-2.0f, 2.0f, 0.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.stats.ring_pairs, 2U);
  EXPECT_EQ(result.stats.rejected_ring_order_pairs, 2U);
  EXPECT_TRUE(result.rays.empty());
}

TEST(ScanStripSupportTest, IsDeterministicAndStrictlySurfaceBounded) {
  std::vector<float> azimuths;
  for (int index = 0; index < 40; ++index) {
    azimuths.push_back(-4.0f + 0.2f * static_cast<float>(index));
  }
  SensorCloud ordered = makeWallSensor(
      {0U, 1U, 2U}, {-2.0f, 0.0f, 2.0f}, azimuths);
  SensorCloud reversed = ordered;
  std::reverse(reversed.points.begin(), reversed.points.end());
  SensorClouds first_input;
  first_input.push_back(ordered);
  SensorClouds second_input;
  second_input.push_back(reversed);
  ScanStripSupportConfig config = testConfig();
  config.max_surface_cells = 7U;

  const ScanStripSupportResult first =
      buildSupportedScanStrips(first_input, config);
  const ScanStripSupportResult second =
      buildSupportedScanStrips(second_input, config);
  ASSERT_TRUE(first.accepted) << first.reason;
  ASSERT_TRUE(second.accepted) << second.reason;
  ASSERT_EQ(first.rays.size(), 7U);
  ASSERT_EQ(second.rays.size(), first.rays.size());
  EXPECT_TRUE(first.surface_budget_limited);
  for (std::size_t index = 0; index < first.rays.size(); ++index) {
    EXPECT_EQ(first.rays[index].sensor_id, second.rays[index].sensor_id);
    EXPECT_EQ(first.rays[index].lower_ring, second.rays[index].lower_ring);
    EXPECT_FLOAT_EQ(first.rays[index].azimuth, second.rays[index].azimuth);
    EXPECT_TRUE(first.rays[index].position.isApprox(
        second.rays[index].position, 1e-6f));
  }
}

TEST(ScanStripSupportTest, CandidateBudgetIsAHardFailSoftLimit) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
                                   {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig config = testConfig();
  config.max_candidate_samples = 5U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.candidate_budget_limited);
  EXPECT_EQ(result.reason, "candidate_budget_limited");
  EXPECT_EQ(result.stats.candidate_samples, 3U);
  EXPECT_LE(result.rays.size(), config.max_surface_cells);
}

TEST(ScanStripSupportTest, ExtremeRasterSpacingFailsBoundedWithoutCasting) {
  SensorClouds sensors;
  sensors.push_back(makeWallSensor({0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
                                   {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}));
  ScanStripSupportConfig config = testConfig();
  config.voxel_size = std::numeric_limits<float>::denorm_min();
  config.max_candidate_samples = 11U;
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.candidate_budget_limited);
  EXPECT_EQ(result.stats.candidate_samples, 0U);
  EXPECT_TRUE(result.rays.empty());
}

TEST(ScanStripSupportTest, CandidateBudgetIsFairAndAtomicAcrossSensors) {
  const std::vector<float> azimuths =
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  SensorCloud first = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f}, azimuths);
  first.sensor_id = 3U;
  SensorCloud second = first;
  second.sensor_id = 9U;

  ScanStripSupportConfig config = testConfig();
  config.max_candidate_samples = 24U;
  SensorClouds ordered{first, second};
  SensorClouds reversed{second, first};
  const ScanStripSupportResult ordered_result =
      buildSupportedScanStrips(ordered, config);
  const ScanStripSupportResult reversed_result =
      buildSupportedScanStrips(reversed, config);
  ASSERT_TRUE(ordered_result.accepted) << ordered_result.reason;
  ASSERT_TRUE(reversed_result.accepted) << reversed_result.reason;
  EXPECT_TRUE(ordered_result.candidate_budget_limited);
  EXPECT_TRUE(reversed_result.candidate_budget_limited);
  EXPECT_LE(ordered_result.stats.candidate_samples,
            config.max_candidate_samples);
  EXPECT_EQ(ordered_result.stats.candidate_samples,
            reversed_result.stats.candidate_samples);

  const auto has_sensor = [](const ScanStripSupportResult& result,
                             std::uint8_t sensor_id) {
    return std::any_of(result.rays.begin(), result.rays.end(),
                       [sensor_id](const SupportedSurfaceRay& ray) {
                         return ray.sensor_id == sensor_id;
                       });
  };
  EXPECT_TRUE(has_sensor(ordered_result, 3U));
  EXPECT_TRUE(has_sensor(ordered_result, 9U));
  EXPECT_TRUE(has_sensor(reversed_result, 3U));
  EXPECT_TRUE(has_sensor(reversed_result, 9U));

  const auto semantic_positions = [](const ScanStripSupportResult& result) {
    std::vector<std::tuple<std::uint8_t, std::uint16_t, int, int, int>> values;
    values.reserve(result.rays.size());
    for (const SupportedSurfaceRay& ray : result.rays) {
      values.emplace_back(ray.sensor_id, ray.lower_ring,
                          static_cast<int>(std::lround(ray.position.x() * 1e5f)),
                          static_cast<int>(std::lround(ray.position.y() * 1e5f)),
                          static_cast<int>(std::lround(ray.position.z() * 1e5f)));
    }
    std::sort(values.begin(), values.end());
    return values;
  };
  EXPECT_EQ(semantic_positions(ordered_result),
            semantic_positions(reversed_result));
}

TEST(ScanStripSupportTest, IgnoresFallbackTopology) {
  SensorCloud sensor = makeWallSensor(
      {0U, 1U, 2U}, {-1.0f, 0.0f, 1.0f},
      {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f});
  for (TsdfPoint& point : sensor.points) {
    point.source_topology_valid = false;
  }
  SensorClouds sensors;
  sensors.push_back(sensor);
  const ScanStripSupportResult result =
      buildSupportedScanStrips(sensors, testConfig());
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.stats.topology_points, 0U);
  EXPECT_TRUE(result.rays.empty());
}

}  // namespace
}  // namespace local_tsdf_mesh

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
