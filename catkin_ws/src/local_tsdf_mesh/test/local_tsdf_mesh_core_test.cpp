#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "local_tsdf_mesh/frame_registrar.h"
#include "local_tsdf_mesh/scan_strip_support.h"
#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {
namespace {

TsdfConfig testTsdfConfig() {
  TsdfConfig config;
  // Existing focused tests retain the historical serial execution order.
  // Dedicated equivalence tests below explicitly enable the worker path.
  config.parallel_support_build = false;
  config.voxel_size = 0.10f;
  config.truncation_distance = 0.30f;
  config.min_range = 0.10f;
  config.max_range = 10.0f;
  config.max_weight_per_voxel_per_frame = 4.0f;
  config.window_duration_sec = 1.0;
  config.max_window_frames = 5;
  config.max_points_per_frame = 10000;
  config.max_voxels = 100000;
  return config;
}

SensorCloud makeSingleRay(std::uint8_t id, const Eigen::Vector3f& origin,
                          const Eigen::Vector3f& surface) {
  SensorCloud sensor;
  sensor.sensor_id = id;
  sensor.origin = origin;
  sensor.points.push_back(surface);
  return sensor;
}

TsdfPoint makeTopologyPoint(std::uint16_t ring, float azimuth, float range,
                            float z = 0.0f) {
  TsdfPoint point;
  point.position = Eigen::Vector3f(range * std::cos(azimuth),
                                   range * std::sin(azimuth), z);
  point.ring = ring;
  point.azimuth = azimuth;
  return point;
}

SensorCloud makeTopologySensor(std::uint8_t id, std::size_t ring_count,
                               std::size_t azimuth_count) {
  SensorCloud sensor;
  sensor.sensor_id = id;
  for (std::size_t ring = 0; ring < ring_count; ++ring) {
    for (std::size_t azimuth_index = 0; azimuth_index < azimuth_count;
         ++azimuth_index) {
      const float azimuth = static_cast<float>(
          6.28318530717958647692 * static_cast<double>(azimuth_index) /
          static_cast<double>(azimuth_count));
      sensor.points.push_back(makeTopologyPoint(
          static_cast<std::uint16_t>(ring), azimuth,
          2.0f + 0.01f * static_cast<float>(azimuth_index),
          0.02f * static_cast<float>(ring)));
    }
  }
  return sensor;
}

SensorCloud makeSupportWallSensor(float wall_x = 5.0f) {
  SensorCloud sensor;
  sensor.sensor_id = 7U;
  const std::vector<float> elevations = {-2.0f, 0.0f, 2.0f};
  const std::vector<float> azimuths = {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f};
  constexpr float kDegreesToRadians =
      3.14159265358979323846f / 180.0f;
  for (std::size_t ring = 0; ring < elevations.size(); ++ring) {
    const float elevation = elevations[ring] * kDegreesToRadians;
    for (const float azimuth_degrees : azimuths) {
      const float azimuth = azimuth_degrees * kDegreesToRadians;
      const float elevation_cosine = std::cos(elevation);
      const Eigen::Vector3f direction(
          elevation_cosine * std::cos(azimuth),
          elevation_cosine * std::sin(azimuth), std::sin(elevation));
      TsdfPoint point(direction * (wall_x / direction.x()));
      point.ring = static_cast<std::uint16_t>(ring);
      point.azimuth = azimuth;
      point.source_topology_valid = true;
      sensor.points.push_back(point);
    }
  }
  return sensor;
}

ScanStripSupportConfig testSupportConfig() {
  ScanStripSupportConfig config;
  config.enabled = true;
  config.min_range = 1.0f;
  config.max_range = 9.0f;
  config.max_surface_cells = 10000U;
  config.max_candidate_samples = 100000U;
  return config;
}

void expectVolumesEqualInSupportWallBounds(const SparseTsdfVolume& left,
                                           const SparseTsdfVolume& right) {
  EXPECT_EQ(left.voxelCount(), right.voxelCount());
  for (int x = 0; x <= 70; ++x) {
    for (int y = -5; y <= 5; ++y) {
      for (int z = -5; z <= 5; ++z) {
        float left_distance = 0.0f;
        float left_weight = 0.0f;
        float right_distance = 0.0f;
        float right_weight = 0.0f;
        const bool left_present = left.voxelValue(
            VoxelKey{x, y, z}, left_distance, left_weight);
        const bool right_present = right.voxelValue(
            VoxelKey{x, y, z}, right_distance, right_weight);
        ASSERT_EQ(left_present, right_present)
            << "voxel=" << x << ',' << y << ',' << z;
        if (left_present) {
          EXPECT_EQ(left_distance, right_distance);
          EXPECT_EQ(left_weight, right_weight);
        }
      }
    }
  }
}

void expectIndexedMeshesExactlyEqual(const IndexedMesh& left,
                                     const IndexedMesh& right) {
  ASSERT_EQ(left.vertices.size(), right.vertices.size());
  for (std::size_t index = 0; index < left.vertices.size(); ++index) {
    EXPECT_EQ(left.vertices[index].x(), right.vertices[index].x())
        << "vertex=" << index << " axis=x";
    EXPECT_EQ(left.vertices[index].y(), right.vertices[index].y())
        << "vertex=" << index << " axis=y";
    EXPECT_EQ(left.vertices[index].z(), right.vertices[index].z())
        << "vertex=" << index << " axis=z";
  }
  ASSERT_EQ(left.triangles.size(), right.triangles.size());
  for (std::size_t index = 0; index < left.triangles.size(); ++index) {
    EXPECT_EQ(left.triangles[index].x(), right.triangles[index].x())
        << "triangle=" << index << " corner=0";
    EXPECT_EQ(left.triangles[index].y(), right.triangles[index].y())
        << "triangle=" << index << " corner=1";
    EXPECT_EQ(left.triangles[index].z(), right.triangles[index].z())
        << "triangle=" << index << " corner=2";
  }
  EXPECT_EQ(left.triangle_limit_reached, right.triangle_limit_reached);
}

void expectIntegrationResultsEquivalent(const IntegrationResult& serial,
                                        const IntegrationResult& parallel) {
  EXPECT_EQ(serial.accepted, parallel.accepted);
  EXPECT_EQ(serial.volume_updated, parallel.volume_updated);
  EXPECT_EQ(serial.volume_committed, parallel.volume_committed);
  EXPECT_EQ(serial.surface_prepared, parallel.surface_prepared);
  EXPECT_EQ(serial.surface_output_accepted, parallel.surface_output_accepted);
  EXPECT_EQ(serial.mesh_published_this_frame,
            parallel.mesh_published_this_frame);
  EXPECT_EQ(serial.reason, parallel.reason);
  EXPECT_EQ(serial.surface_output_reason, parallel.surface_output_reason);
  EXPECT_EQ(serial.surface_mode, parallel.surface_mode);
  EXPECT_EQ(serial.measured_reason, parallel.measured_reason);
  EXPECT_EQ(serial.measured_capacity_limited,
            parallel.measured_capacity_limited);
  EXPECT_EQ(serial.input_points, parallel.input_points);
  EXPECT_EQ(serial.sampled_points, parallel.sampled_points);
  EXPECT_EQ(serial.integrated_rays, parallel.integrated_rays);
  EXPECT_EQ(serial.contributed_voxels, parallel.contributed_voxels);
  EXPECT_EQ(serial.evicted_frames, parallel.evicted_frames);
  EXPECT_EQ(serial.active_frames, parallel.active_frames);
  EXPECT_EQ(serial.active_voxels, parallel.active_voxels);
  EXPECT_EQ(serial.support_reason, parallel.support_reason);
  EXPECT_EQ(serial.support_candidate_budget_limited,
            parallel.support_candidate_budget_limited);
  EXPECT_EQ(serial.support_surface_budget_limited,
            parallel.support_surface_budget_limited);
  EXPECT_EQ(serial.support_topology_points, parallel.support_topology_points);
  EXPECT_EQ(serial.support_candidate_quads, parallel.support_candidate_quads);
  EXPECT_EQ(serial.support_accepted_quads, parallel.support_accepted_quads);
  EXPECT_EQ(serial.support_input_points, parallel.support_input_points);
  EXPECT_EQ(serial.support_ring_pairs, parallel.support_ring_pairs);
  EXPECT_EQ(serial.support_rejected_ring_pairs,
            parallel.support_rejected_ring_pairs);
  EXPECT_EQ(serial.support_rejected_ring_order_pairs,
            parallel.support_rejected_ring_order_pairs);
  EXPECT_EQ(serial.support_locally_valid_quads,
            parallel.support_locally_valid_quads);
  EXPECT_EQ(serial.support_strong_quads, parallel.support_strong_quads);
  EXPECT_EQ(serial.support_verified_long_quads,
            parallel.support_verified_long_quads);
  EXPECT_EQ(serial.support_rejected_long_quads,
            parallel.support_rejected_long_quads);
  EXPECT_EQ(serial.support_rejected_isolated_quads,
            parallel.support_rejected_isolated_quads);
  EXPECT_EQ(serial.support_candidate_samples,
            parallel.support_candidate_samples);
  EXPECT_EQ(serial.support_surface_rays, parallel.support_surface_rays);
  EXPECT_EQ(serial.support_integrated_rays, parallel.support_integrated_rays);
  EXPECT_EQ(serial.support_contributed_voxels,
            parallel.support_contributed_voxels);
  EXPECT_EQ(serial.support_mesh_reason, parallel.support_mesh_reason);
  EXPECT_EQ(serial.support_mesh_apply_reason,
            parallel.support_mesh_apply_reason);
  EXPECT_EQ(serial.support_mesh_budget_limited,
            parallel.support_mesh_budget_limited);
  EXPECT_EQ(serial.support_mesh_accepted, parallel.support_mesh_accepted);
  EXPECT_EQ(serial.support_mesh_applied, parallel.support_mesh_applied);
  EXPECT_EQ(serial.support_mesh_vertices, parallel.support_mesh_vertices);
  EXPECT_EQ(serial.support_mesh_triangles, parallel.support_mesh_triangles);
  EXPECT_EQ(serial.support_mesh_curve_intervals,
            parallel.support_mesh_curve_intervals);
  EXPECT_EQ(serial.support_mesh_skipped_curve_intervals,
            parallel.support_mesh_skipped_curve_intervals);
  EXPECT_EQ(serial.support_mesh_skipped_degenerate_intervals,
            parallel.support_mesh_skipped_degenerate_intervals);
  EXPECT_EQ(serial.support_mesh_skipped_sensor_columns,
            parallel.support_mesh_skipped_sensor_columns);
  EXPECT_EQ(serial.support_mesh_output_equivalence_input_triangles,
            parallel.support_mesh_output_equivalence_input_triangles);
  EXPECT_EQ(serial.support_mesh_output_equivalence_removed_triangles,
            parallel.support_mesh_output_equivalence_removed_triangles);
  EXPECT_EQ(serial.support_mesh_output_equivalence_masked_sensor_columns,
            parallel.support_mesh_output_equivalence_masked_sensor_columns);
  EXPECT_EQ(serial.support_mesh_has_first_curve_gap,
            parallel.support_mesh_has_first_curve_gap);
  EXPECT_EQ(serial.support_mesh_first_curve_gap_sensor_id,
            parallel.support_mesh_first_curve_gap_sensor_id);
  EXPECT_EQ(serial.support_mesh_first_curve_gap_lower_ring,
            parallel.support_mesh_first_curve_gap_lower_ring);
  EXPECT_EQ(serial.support_mesh_first_curve_gap_start_azimuth,
            parallel.support_mesh_first_curve_gap_start_azimuth);
  EXPECT_EQ(serial.support_mesh_first_curve_gap_end_azimuth,
            parallel.support_mesh_first_curve_gap_end_azimuth);
  EXPECT_EQ(serial.support_mesh_first_curve_gap_corner,
            parallel.support_mesh_first_curve_gap_corner);
  expectIndexedMeshesExactlyEqual(serial.support_mesh,
                                  parallel.support_mesh);
}

std::tuple<unsigned int, std::uint16_t, int> selectionKey(
    const SensorClouds& sensors, const RaySelection& selection) {
  const SensorCloud& sensor = sensors[selection.sensor_index];
  const TsdfPoint& point = sensor.points[selection.point_index];
  return std::make_tuple(
      static_cast<unsigned int>(sensor.sensor_id), point.ring,
      static_cast<int>(std::lround(point.azimuth * 1000000.0f)));
}

TEST(SparseTsdfVolumeTest, RejectsInvalidConfiguration) {
  TsdfConfig config = testTsdfConfig();
  config.voxel_size = 0.0f;
  EXPECT_THROW(SparseTsdfVolume volume(config), std::invalid_argument);

  config = testTsdfConfig();
  config.truncation_distance = std::numeric_limits<float>::infinity();
  EXPECT_THROW(SparseTsdfVolume volume(config), std::invalid_argument);

  config = testTsdfConfig();
  config.min_range = 0.0f;
  EXPECT_THROW(SparseTsdfVolume volume(config), std::invalid_argument);

  config = testTsdfConfig();
  config.truncation_distance = config.voxel_size * 100.0f;
  EXPECT_THROW(SparseTsdfVolume volume(config), std::invalid_argument);
}

TEST(SparseTsdfVolumeTest, IntegratesPositiveAndNegativeSidesOfSurface) {
  SparseTsdfVolume volume(testTsdfConfig());
  SensorClouds sensors;
  sensors.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                  Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  const IntegrationResult result = volume.integrateFrame(sensors, 1.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.volume_updated);
  EXPECT_EQ(volume.frameCount(), 1U);

  float front_distance = 0.0f;
  float front_weight = 0.0f;
  float behind_distance = 0.0f;
  float behind_weight = 0.0f;
  ASSERT_TRUE(volume.voxelValue(VoxelKey{8, 0, 0}, front_distance,
                                front_weight));
  ASSERT_TRUE(volume.voxelValue(VoxelKey{11, 0, 0}, behind_distance,
                                behind_weight));
  EXPECT_GT(front_distance, 0.0f);
  EXPECT_LT(behind_distance, 0.0f);
  EXPECT_GT(front_weight, 0.0f);
  EXPECT_GT(behind_weight, 0.0f);
}

TEST(SparseTsdfVolumeTest, DisabledScanSupportPreservesBaseIntegration) {
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  SparseTsdfVolume implicit_disabled(testTsdfConfig());
  SparseTsdfVolume explicit_disabled(testTsdfConfig());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.enabled = false;

  const IntegrationResult implicit_result =
      implicit_disabled.integrateFrame(sensors, 1.0);
  const IntegrationResult explicit_result =
      explicit_disabled.integrateFrame(sensors, 1.0, support_config);
  ASSERT_TRUE(implicit_result.accepted) << implicit_result.reason;
  ASSERT_TRUE(explicit_result.accepted) << explicit_result.reason;
  EXPECT_EQ(implicit_result.reason, explicit_result.reason);
  EXPECT_EQ(implicit_result.input_points, explicit_result.input_points);
  EXPECT_EQ(implicit_result.sampled_points, explicit_result.sampled_points);
  EXPECT_EQ(implicit_result.integrated_rays, explicit_result.integrated_rays);
  EXPECT_EQ(implicit_result.contributed_voxels,
            explicit_result.contributed_voxels);
  EXPECT_EQ(implicit_result.active_frames, explicit_result.active_frames);
  EXPECT_EQ(implicit_result.active_voxels, explicit_result.active_voxels);
  EXPECT_EQ(implicit_result.support_reason, "disabled");
  EXPECT_EQ(explicit_result.support_reason, "disabled");
  EXPECT_EQ(explicit_result.support_surface_rays, 0U);
  EXPECT_EQ(explicit_result.support_contributed_voxels, 0U);
  expectVolumesEqualInSupportWallBounds(implicit_disabled,
                                        explicit_disabled);
}

TEST(SparseTsdfVolumeTest, ScanSupportAddsBoundedLowWeightSurfaceBand) {
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  SparseTsdfVolume base_volume(testTsdfConfig());
  SparseTsdfVolume support_volume(testTsdfConfig());
  const IntegrationResult base_result =
      base_volume.integrateFrame(sensors, 1.0);
  const ScanStripSupportConfig support_config = testSupportConfig();
  const IntegrationResult support_result =
      support_volume.integrateFrame(sensors, 1.0, support_config);
  ASSERT_TRUE(base_result.accepted) << base_result.reason;
  ASSERT_TRUE(support_result.accepted) << support_result.reason;
  EXPECT_EQ(support_result.support_reason, "accepted");
  EXPECT_EQ(support_result.support_topology_points, sensors[0].points.size());
  EXPECT_GT(support_result.support_accepted_quads, 0U);
  EXPECT_GT(support_result.support_surface_rays, 0U);
  EXPECT_EQ(support_result.support_integrated_rays,
            support_result.support_surface_rays);
  EXPECT_GT(support_result.support_contributed_voxels, 0U);
  EXPECT_GT(support_result.contributed_voxels,
            base_result.contributed_voxels);

  bool found_support_weight = false;
  for (int x = 0; x <= 70; ++x) {
    for (int y = -5; y <= 5; ++y) {
      for (int z = -5; z <= 5; ++z) {
        const VoxelKey key{x, y, z};
        float base_distance = 0.0f;
        float base_weight = 0.0f;
        float support_distance = 0.0f;
        float support_weight = 0.0f;
        const bool base_present = base_volume.voxelValue(
            key, base_distance, base_weight);
        if (!support_volume.voxelValue(
                key, support_distance, support_weight)) {
          continue;
        }
        const float added_weight =
            support_weight - (base_present ? base_weight : 0.0f);
        EXPECT_LE(added_weight,
                  support_config.maximum_weight_per_voxel + 1e-5f);
        if (added_weight > 1e-5f) {
          found_support_weight = true;
        }
      }
    }
  }
  EXPECT_TRUE(found_support_weight);
}

TEST(SparseTsdfVolumeTest,
     IndexedSupportWithoutSurfaceRaysPreservesBaseIntegration) {
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  SparseTsdfVolume base_volume(testTsdfConfig());
  SparseTsdfVolume indexed_volume(testTsdfConfig());
  const IntegrationResult base_result =
      base_volume.integrateFrame(sensors, 1.0);
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = false;
  support_config.build_indexed_mesh = true;
  const IntegrationResult indexed_result =
      indexed_volume.integrateFrame(sensors, 1.0, support_config);

  ASSERT_TRUE(base_result.accepted) << base_result.reason;
  ASSERT_TRUE(indexed_result.accepted) << indexed_result.reason;
  EXPECT_EQ(indexed_result.support_reason, "surface_rays_disabled");
  EXPECT_EQ(indexed_result.support_candidate_samples, 0U);
  EXPECT_EQ(indexed_result.support_surface_rays, 0U);
  EXPECT_EQ(indexed_result.support_integrated_rays, 0U);
  EXPECT_EQ(indexed_result.support_contributed_voxels, 0U);
  EXPECT_EQ(indexed_result.support_mesh_reason, "accepted");
  EXPECT_FALSE(indexed_result.support_mesh_budget_limited);
  EXPECT_GT(indexed_result.support_mesh_vertices, 0U);
  EXPECT_GT(indexed_result.support_mesh_triangles, 0U);
  EXPECT_EQ(indexed_result.support_mesh_vertices,
            indexed_result.support_mesh.vertices.size());
  EXPECT_EQ(indexed_result.support_mesh_triangles,
            indexed_result.support_mesh.triangles.size());
  EXPECT_EQ(base_result.contributed_voxels,
            indexed_result.contributed_voxels);
  expectVolumesEqualInSupportWallBounds(base_volume, indexed_volume);
}

TEST(SparseTsdfVolumeTest,
     ParallelSupportBuildMatchesSerialAcrossConsecutiveFrames) {
  TsdfConfig serial_config = testTsdfConfig();
  serial_config.parallel_support_build = false;
  TsdfConfig parallel_config = serial_config;
  parallel_config.parallel_support_build = true;
  SparseTsdfVolume serial_volume(serial_config);
  SparseTsdfVolume parallel_volume(parallel_config);

  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = true;
  support_config.build_indexed_mesh = true;
  SensorClouds first_sensors;
  first_sensors.push_back(makeSupportWallSensor(5.0f));
  SensorClouds second_sensors;
  second_sensors.push_back(makeSupportWallSensor(4.8f));

  const std::vector<SensorClouds> frames = {first_sensors, second_sensors};
  const std::vector<double> stamps = {1.0, 1.2};
  for (std::size_t frame = 0; frame < frames.size(); ++frame) {
    const IntegrationResult serial_result = serial_volume.integrateFrame(
        frames[frame], stamps[frame], support_config);
    const IntegrationResult parallel_result = parallel_volume.integrateFrame(
        frames[frame], stamps[frame], support_config);
    ASSERT_TRUE(serial_result.accepted) << serial_result.reason;
    ASSERT_TRUE(parallel_result.accepted) << parallel_result.reason;
    ASSERT_TRUE(serial_result.support_mesh_accepted)
        << serial_result.support_mesh_reason;
    ASSERT_GT(serial_result.support_mesh_triangles, 0U);
    ASSERT_GT(serial_result.support_surface_rays, 0U);
    ASSERT_GT(serial_result.support_contributed_voxels, 0U);
    EXPECT_FALSE(serial_result.support_build_parallel);
    EXPECT_TRUE(parallel_result.support_build_parallel);
    expectIntegrationResultsEquivalent(serial_result, parallel_result);
    expectVolumesEqualInSupportWallBounds(serial_volume, parallel_volume);
    EXPECT_EQ(serial_volume.frameCount(), parallel_volume.frameCount());
    EXPECT_EQ(serial_volume.voxelCount(), parallel_volume.voxelCount());
    EXPECT_DOUBLE_EQ(serial_volume.latestStamp(), parallel_volume.latestStamp());
    expectIndexedMeshesExactlyEqual(
        serial_volume.extractMesh(MeshExtractionConfig()),
        parallel_volume.extractMesh(MeshExtractionConfig()));
  }
}

TEST(SparseTsdfVolumeTest,
     ParallelSupportMatchesMeasuredCapacityDirectFallback) {
  TsdfConfig serial_config = testTsdfConfig();
  serial_config.max_voxels = 16U;
  serial_config.parallel_support_build = false;
  TsdfConfig parallel_config = serial_config;
  parallel_config.parallel_support_build = true;
  SparseTsdfVolume serial_volume(serial_config);
  SparseTsdfVolume parallel_volume(parallel_config);

  SensorClouds seed;
  seed.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                               Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  ASSERT_TRUE(serial_volume.integrateFrame(seed, 0.25).accepted);
  ASSERT_TRUE(parallel_volume.integrateFrame(seed, 0.25).accepted);

  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = false;
  support_config.build_indexed_mesh = true;
  const IntegrationResult serial_result =
      serial_volume.integrateFrame(sensors, 5.0, support_config);
  const IntegrationResult parallel_result =
      parallel_volume.integrateFrame(sensors, 5.0, support_config);

  ASSERT_TRUE(serial_result.accepted) << serial_result.reason;
  ASSERT_TRUE(parallel_result.accepted) << parallel_result.reason;
  EXPECT_EQ(serial_result.reason, "direct_surface_only");
  EXPECT_TRUE(serial_result.measured_capacity_limited);
  EXPECT_FALSE(serial_result.support_build_parallel);
  EXPECT_TRUE(parallel_result.support_build_parallel);
  expectIntegrationResultsEquivalent(serial_result, parallel_result);
  expectVolumesEqualInSupportWallBounds(serial_volume, parallel_volume);
  EXPECT_EQ(serial_volume.frameCount(), parallel_volume.frameCount());
  EXPECT_EQ(serial_volume.voxelCount(), parallel_volume.voxelCount());
  EXPECT_DOUBLE_EQ(serial_volume.latestStamp(), parallel_volume.latestStamp());
}

TEST(SparseTsdfVolumeTest,
     ParallelSupportDoesNotLaunchForInvalidOrEmptyFrames) {
  TsdfConfig config = testTsdfConfig();
  config.parallel_support_build = true;
  SparseTsdfVolume volume(config);
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_indexed_mesh = true;
  SensorClouds valid_sensors;
  valid_sensors.push_back(makeSupportWallSensor());

  const IntegrationResult invalid_stamp = volume.integrateFrame(
      valid_sensors, std::numeric_limits<double>::quiet_NaN(), support_config);
  EXPECT_FALSE(invalid_stamp.accepted);
  EXPECT_EQ(invalid_stamp.reason, "invalid_stamp");
  EXPECT_FALSE(invalid_stamp.support_build_parallel);
  EXPECT_DOUBLE_EQ(invalid_stamp.support_build_ms, 0.0);

  const IntegrationResult empty =
      volume.integrateFrame(SensorClouds(), 1.0, support_config);
  EXPECT_FALSE(empty.accepted);
  EXPECT_EQ(empty.reason, "empty_frame");
  EXPECT_FALSE(empty.support_build_parallel);
  EXPECT_DOUBLE_EQ(empty.support_build_ms, 0.0);

  SensorClouds zero_points(1);
  const IntegrationResult zero_point_frame =
      volume.integrateFrame(zero_points, 1.0, support_config);
  EXPECT_FALSE(zero_point_frame.accepted);
  EXPECT_EQ(zero_point_frame.reason, "empty_frame");
  EXPECT_FALSE(zero_point_frame.support_build_parallel);
  EXPECT_DOUBLE_EQ(zero_point_frame.support_build_ms, 0.0);
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
}

TEST(SparseTsdfVolumeTest,
     DirectIndexedSupportCanRunWithoutMeasuredTsdfContribution) {
  TsdfConfig config = testTsdfConfig();
  config.integrate_measured_rays = false;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = false;
  support_config.build_indexed_mesh = true;

  const IntegrationResult first =
      volume.integrateFrame(sensors, 1.0, support_config);
  ASSERT_TRUE(first.accepted) << first.reason;
  EXPECT_EQ(first.reason, "direct_surface_only");
  EXPECT_FALSE(first.volume_updated);
  EXPECT_TRUE(first.support_mesh_accepted);
  EXPECT_GT(first.support_mesh_triangles, 0U);
  EXPECT_EQ(first.sampled_points, 0U);
  EXPECT_EQ(first.integrated_rays, 0U);
  EXPECT_EQ(first.contributed_voxels, 0U);
  EXPECT_EQ(first.active_frames, 0U);
  EXPECT_EQ(first.active_voxels, 0U);
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
  EXPECT_LT(volume.latestStamp(), 0.0);
  EXPECT_TRUE(volume.extractMesh(MeshExtractionConfig()).triangles.empty());

  const IntegrationResult second =
      volume.integrateFrame(sensors, 2.0, support_config);
  ASSERT_TRUE(second.accepted) << second.reason;
  EXPECT_EQ(second.reason, "direct_surface_only");
  EXPECT_FALSE(second.volume_updated);
  EXPECT_TRUE(second.support_mesh_accepted);
  EXPECT_GT(second.support_mesh_triangles, 0U);
  EXPECT_EQ(second.active_frames, 0U);
  EXPECT_EQ(second.active_voxels, 0U);
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
  EXPECT_LT(volume.latestStamp(), 0.0);
}

TEST(SparseTsdfVolumeTest,
     DisabledMeasuredRaysStillAllowSupportSurfaceTsdfIntegration) {
  TsdfConfig config = testTsdfConfig();
  config.integrate_measured_rays = false;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = true;
  support_config.build_indexed_mesh = false;

  const IntegrationResult result =
      volume.integrateFrame(sensors, 1.0, support_config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "accepted");
  EXPECT_EQ(result.measured_reason, "disabled");
  EXPECT_TRUE(result.volume_updated);
  EXPECT_EQ(result.sampled_points, 0U);
  EXPECT_EQ(result.integrated_rays, 0U);
  EXPECT_GT(result.support_surface_rays, 0U);
  EXPECT_EQ(result.support_integrated_rays, result.support_surface_rays);
  EXPECT_GT(result.support_contributed_voxels, 0U);
  EXPECT_EQ(result.contributed_voxels,
            result.support_contributed_voxels);
  EXPECT_EQ(volume.frameCount(), 1U);
  EXPECT_GT(volume.voxelCount(), 0U);
  EXPECT_DOUBLE_EQ(volume.latestStamp(), 1.0);
}

TEST(SparseTsdfVolumeTest,
     DirectOnlyModeFailsClosedWithoutAnIndexedSurface) {
  TsdfConfig config = testTsdfConfig();
  config.integrate_measured_rays = false;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.enabled = false;

  const IntegrationResult result =
      volume.integrateFrame(sensors, 1.0, support_config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "no_valid_rays");
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
  EXPECT_LT(volume.latestStamp(), 0.0);
}

TEST(SparseTsdfVolumeTest,
     FarDirectSurfaceSurvivesAFrameWithoutNearMeasuredRays) {
  TsdfConfig config = testTsdfConfig();
  config.max_range = 1.0f;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = false;
  support_config.build_indexed_mesh = true;

  const IntegrationResult result =
      volume.integrateFrame(sensors, 1.0, support_config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "direct_surface_only");
  EXPECT_FALSE(result.volume_updated);
  EXPECT_EQ(result.integrated_rays, 0U);
  EXPECT_GT(result.support_mesh_triangles, 0U);
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
}

TEST(SparseTsdfVolumeTest,
     DirectSurfaceSurvivesAnOversizedMeasuredContribution) {
  TsdfConfig config = testTsdfConfig();
  config.max_voxels = 16U;
  SparseTsdfVolume volume(config);
  SparseTsdfVolume expected_volume(config);
  SensorClouds seed;
  seed.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                               Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  ASSERT_TRUE(volume.integrateFrame(seed, 0.25).accepted);
  ASSERT_TRUE(expected_volume.integrateFrame(seed, 0.25).accepted);
  const std::size_t seed_frames = volume.frameCount();
  const std::size_t seed_voxels = volume.voxelCount();

  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.build_surface_rays = false;
  support_config.build_indexed_mesh = true;

  const IntegrationResult result =
      volume.integrateFrame(sensors, 5.0, support_config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "direct_surface_only");
  EXPECT_EQ(result.measured_reason, "frame_exceeds_voxel_capacity");
  EXPECT_TRUE(result.measured_capacity_limited);
  EXPECT_FALSE(result.volume_updated);
  EXPECT_EQ(result.integrated_rays, 0U);
  EXPECT_EQ(result.contributed_voxels, 0U);
  EXPECT_TRUE(result.support_mesh_accepted);
  EXPECT_GT(result.support_mesh_triangles, 0U);
  EXPECT_EQ(result.active_frames, seed_frames);
  EXPECT_EQ(result.active_voxels, seed_voxels);
  EXPECT_EQ(volume.frameCount(), seed_frames);
  EXPECT_EQ(volume.voxelCount(), seed_voxels);
  EXPECT_DOUBLE_EQ(volume.latestStamp(), 0.25);
  expectVolumesEqualInSupportWallBounds(volume, expected_volume);
}

TEST(SparseTsdfVolumeTest,
     OversizedMeasuredContributionWithoutDirectSurfaceFailsClosed) {
  TsdfConfig config = testTsdfConfig();
  config.max_voxels = 16U;
  SparseTsdfVolume volume(config);
  SparseTsdfVolume expected_volume(config);
  SensorClouds seed;
  seed.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                               Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  ASSERT_TRUE(volume.integrateFrame(seed, 0.25).accepted);
  ASSERT_TRUE(expected_volume.integrateFrame(seed, 0.25).accepted);
  const std::size_t seed_frames = volume.frameCount();
  const std::size_t seed_voxels = volume.voxelCount();

  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  ScanStripSupportConfig support_config = testSupportConfig();
  support_config.enabled = false;

  const IntegrationResult result =
      volume.integrateFrame(sensors, 5.0, support_config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "frame_exceeds_voxel_capacity");
  EXPECT_EQ(result.measured_reason, "frame_exceeds_voxel_capacity");
  EXPECT_TRUE(result.measured_capacity_limited);
  EXPECT_FALSE(result.volume_updated);
  EXPECT_EQ(result.integrated_rays, 0U);
  EXPECT_EQ(result.contributed_voxels, 0U);
  EXPECT_EQ(result.active_frames, seed_frames);
  EXPECT_EQ(result.active_voxels, seed_voxels);
  EXPECT_EQ(volume.frameCount(), seed_frames);
  EXPECT_EQ(volume.voxelCount(), seed_voxels);
  EXPECT_DOUBLE_EQ(volume.latestStamp(), 0.25);
  expectVolumesEqualInSupportWallBounds(volume, expected_volume);
}

TEST(IndexedMeshAppendTest, OffsetsIndicesAndFailsSoftAtTotalCapacity) {
  IndexedMesh base;
  base.vertices = {Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                   Eigen::Vector3f(0.0f, 1.0f, 0.0f)};
  base.triangles = {Eigen::Vector3i(0, 1, 2)};
  IndexedMesh addition;
  addition.vertices = {Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                       Eigen::Vector3f(1.0f, 0.0f, 1.0f),
                       Eigen::Vector3f(0.0f, 1.0f, 1.0f)};
  addition.triangles = {Eigen::Vector3i(0, 2, 1)};

  IndexedMesh appended = base;
  const MeshAppendResult success =
      appendIndexedMeshAtomic(addition, 2U, appended);
  ASSERT_TRUE(success.applied) << success.reason;
  EXPECT_FALSE(success.budget_limited);
  EXPECT_EQ(success.appended_vertices, 3U);
  EXPECT_EQ(success.appended_triangles, 1U);
  ASSERT_EQ(appended.vertices.size(), 6U);
  ASSERT_EQ(appended.triangles.size(), 2U);
  EXPECT_EQ(appended.triangles[0].x(), 0);
  EXPECT_EQ(appended.triangles[0].y(), 1);
  EXPECT_EQ(appended.triangles[0].z(), 2);
  EXPECT_EQ(appended.triangles[1].x(), 3);
  EXPECT_EQ(appended.triangles[1].y(), 5);
  EXPECT_EQ(appended.triangles[1].z(), 4);

  IndexedMesh capacity_limited = base;
  const MeshAppendResult rejected =
      appendIndexedMeshAtomic(std::move(addition), 1U, capacity_limited);
  EXPECT_FALSE(rejected.applied);
  EXPECT_TRUE(rejected.budget_limited);
  EXPECT_EQ(rejected.reason, "total_triangle_capacity");
  ASSERT_EQ(capacity_limited.vertices.size(), 3U);
  ASSERT_EQ(capacity_limited.triangles.size(), 1U);
  EXPECT_TRUE(capacity_limited.vertices[0].isApprox(base.vertices[0], 0.0f));
  EXPECT_TRUE(capacity_limited.vertices[1].isApprox(base.vertices[1], 0.0f));
  EXPECT_TRUE(capacity_limited.vertices[2].isApprox(base.vertices[2], 0.0f));
  EXPECT_EQ(capacity_limited.triangles[0].x(), base.triangles[0].x());
  EXPECT_EQ(capacity_limited.triangles[0].y(), base.triangles[0].y());
  EXPECT_EQ(capacity_limited.triangles[0].z(), base.triangles[0].z());
}

TEST(SparseTsdfVolumeTest, SupportCapacityFailureLeavesBaseContributionIntact) {
  SensorClouds sensors;
  sensors.push_back(makeSupportWallSensor());
  TsdfConfig roomy_config = testTsdfConfig();
  SparseTsdfVolume sizing_volume(roomy_config);
  const IntegrationResult sizing_result =
      sizing_volume.integrateFrame(sensors, 1.0);
  ASSERT_TRUE(sizing_result.accepted) << sizing_result.reason;
  ASSERT_GT(sizing_result.contributed_voxels, 0U);

  TsdfConfig tight_config = roomy_config;
  tight_config.max_voxels = sizing_result.contributed_voxels;
  SparseTsdfVolume base_volume(tight_config);
  SparseTsdfVolume support_volume(tight_config);
  const IntegrationResult base_result =
      base_volume.integrateFrame(sensors, 1.0);
  const IntegrationResult support_result = support_volume.integrateFrame(
      sensors, 1.0, testSupportConfig());
  ASSERT_TRUE(base_result.accepted) << base_result.reason;
  ASSERT_TRUE(support_result.accepted) << support_result.reason;
  EXPECT_EQ(support_result.support_reason, "support_voxel_capacity");
  EXPECT_GT(support_result.support_surface_rays, 0U);
  EXPECT_EQ(support_result.support_integrated_rays, 0U);
  EXPECT_EQ(support_result.support_contributed_voxels, 0U);
  EXPECT_EQ(base_result.contributed_voxels,
            support_result.contributed_voxels);
  expectVolumesEqualInSupportWallBounds(base_volume, support_volume);
}

TEST(StratifiedRaySamplerTest, IsDeterministicAndStrictlyBounded) {
  SensorClouds sensors;
  sensors.push_back(makeTopologySensor(0, 4, 12));
  sensors.push_back(makeTopologySensor(1, 3, 8));

  const std::vector<RaySelection> first =
      selectStratifiedRays(sensors, 17, 0.1f, 10.0f);
  const std::vector<RaySelection> second =
      selectStratifiedRays(sensors, 17, 0.1f, 10.0f);
  EXPECT_EQ(first, second);
  EXPECT_EQ(first.size(), 17U);
  EXPECT_EQ(selectStratifiedRays(sensors, 0, 0.1f, 10.0f).size(), 0U);
  EXPECT_EQ(selectStratifiedRays(sensors, 1000, 0.1f, 10.0f).size(),
            72U);
}

TEST(StratifiedRaySamplerTest, LargerBudgetsAreExactNestedPrefixes) {
  SensorClouds sensors;
  sensors.push_back(makeTopologySensor(0, 3, 7));
  sensors.push_back(makeTopologySensor(1, 2, 9));
  const std::vector<RaySelection> complete =
      selectStratifiedRays(sensors, 1000, 0.1f, 10.0f);
  ASSERT_EQ(complete.size(), 39U);
  for (std::size_t budget = 0; budget <= complete.size(); ++budget) {
    const std::vector<RaySelection> selected =
        selectStratifiedRays(sensors, budget, 0.1f, 10.0f);
    ASSERT_EQ(selected.size(), budget) << "budget=" << budget;
    EXPECT_TRUE(std::equal(selected.begin(), selected.end(), complete.begin()))
        << "budget=" << budget;
  }
}

TEST(StratifiedRaySamplerTest, TinyBudgetsRoundRobinAcrossLidars) {
  SensorClouds sensors;
  sensors.push_back(makeTopologySensor(0, 4, 20));
  sensors.push_back(makeTopologySensor(1, 2, 3));

  const std::vector<RaySelection> two =
      selectStratifiedRays(sensors, 2, 0.1f, 10.0f);
  ASSERT_EQ(two.size(), 2U);
  EXPECT_EQ(sensors[two[0].sensor_index].sensor_id, 0U);
  EXPECT_EQ(sensors[two[1].sensor_index].sensor_id, 1U);

  const std::vector<RaySelection> three =
      selectStratifiedRays(sensors, 3, 0.1f, 10.0f);
  std::map<std::uint8_t, std::size_t> counts;
  for (const RaySelection& selection : three) {
    ++counts[sensors[selection.sensor_index].sensor_id];
  }
  EXPECT_EQ(counts[0], 2U);
  EXPECT_EQ(counts[1], 1U);
}

TEST(StratifiedRaySamplerTest, SmallBudgetsCoverRingsAndOppositeSectors) {
  SensorClouds sensors;
  sensors.push_back(makeTopologySensor(0, 4, 4));

  const std::vector<RaySelection> first_round =
      selectStratifiedRays(sensors, 4, 0.1f, 10.0f);
  std::set<std::uint16_t> rings;
  for (const RaySelection& selection : first_round) {
    rings.insert(sensors[selection.sensor_index]
                     .points[selection.point_index]
                     .ring);
  }
  EXPECT_EQ(rings.size(), 4U);

  const std::vector<RaySelection> second_round =
      selectStratifiedRays(sensors, 8, 0.1f, 10.0f);
  std::map<std::uint16_t, std::vector<float>> azimuths;
  for (const RaySelection& selection : second_round) {
    const TsdfPoint& point =
        sensors[selection.sensor_index].points[selection.point_index];
    azimuths[point.ring].push_back(point.azimuth);
  }
  ASSERT_EQ(azimuths.size(), 4U);
  for (const auto& ring : azimuths) {
    ASSERT_EQ(ring.second.size(), 2U);
    float difference = std::fabs(ring.second[0] - ring.second[1]);
    difference = std::min(difference,
                          static_cast<float>(6.28318530717958647692) -
                              difference);
    EXPECT_NEAR(difference, static_cast<float>(3.14159265358979323846),
                0.02f);
  }
}

TEST(StratifiedRaySamplerTest, SparseSectorOrderMatchesExactFarthestReference) {
  const std::vector<std::size_t> sectors = {
      3U,  11U, 25U, 47U, 89U, 121U, 167U,
      191U, 238U, 271U, 313U, 347U};
  SensorCloud sensor;
  sensor.sensor_id = 0;
  for (const std::size_t sector : sectors) {
    const float azimuth = static_cast<float>(
        (static_cast<double>(sector) + 0.25) *
        6.28318530717958647692 / 360.0);
    sensor.points.push_back(makeTopologyPoint(4, azimuth, 2.0f));
  }
  SensorClouds sensors;
  sensors.push_back(sensor);

  std::vector<std::size_t> expected_indices;
  std::set<std::size_t> chosen;
  expected_indices.push_back(0U);
  chosen.insert(0U);
  while (expected_indices.size() < sectors.size()) {
    std::size_t best_index = 0;
    std::size_t best_distance = 0;
    bool have_best = false;
    for (std::size_t candidate = 0; candidate < sectors.size(); ++candidate) {
      if (chosen.count(candidate) != 0U) {
        continue;
      }
      std::size_t minimum_distance = 360U;
      for (const std::size_t selected : chosen) {
        const std::size_t direct = sectors[candidate] > sectors[selected]
                                       ? sectors[candidate] - sectors[selected]
                                       : sectors[selected] - sectors[candidate];
        minimum_distance =
            std::min(minimum_distance, std::min(direct, 360U - direct));
      }
      if (!have_best || minimum_distance > best_distance ||
          (minimum_distance == best_distance &&
           sectors[candidate] < sectors[best_index])) {
        best_index = candidate;
        best_distance = minimum_distance;
        have_best = true;
      }
    }
    chosen.insert(best_index);
    expected_indices.push_back(best_index);
  }

  const std::vector<RaySelection> actual =
      selectStratifiedRays(sensors, sectors.size(), 0.1f, 10.0f);
  ASSERT_EQ(actual.size(), expected_indices.size());
  for (std::size_t order = 0; order < actual.size(); ++order) {
    EXPECT_EQ(actual[order].point_index, expected_indices[order])
        << "order=" << order;
  }
}

TEST(StratifiedRaySamplerTest, UsesSourceAzimuthInsteadOfVolumeGeometry) {
  SensorCloud sensor;
  sensor.sensor_id = 0;
  for (int quadrant = 0; quadrant < 4; ++quadrant) {
    const float azimuth = static_cast<float>(
        quadrant * 3.14159265358979323846 / 2.0);
    TsdfPoint point;
    // Simulate a cloud that has already been rigidly transformed: source
    // azimuth remains the authoritative scan-topology coordinate.
    point.position = Eigen::Vector3f(2.0f, 0.01f * quadrant, 0.0f);
    point.ring = 3;
    point.azimuth = azimuth;
    sensor.points.push_back(point);
  }
  SensorClouds sensors;
  sensors.push_back(sensor);

  const std::vector<RaySelection> selected =
      selectStratifiedRays(sensors, 2, 0.1f, 10.0f);
  ASSERT_EQ(selected.size(), 2U);
  const float first = sensors[0].points[selected[0].point_index].azimuth;
  const float second = sensors[0].points[selected[1].point_index].azimuth;
  float difference = std::fabs(first - second);
  difference = std::min(difference,
                        static_cast<float>(6.28318530717958647692) -
                            difference);
  EXPECT_NEAR(difference, static_cast<float>(3.14159265358979323846),
              0.02f);
}

TEST(StratifiedRaySamplerTest, InvalidRaysDoNotConsumeTheBudget) {
  SensorCloud invalid_origin = makeTopologySensor(0, 1, 4);
  invalid_origin.origin.x() = std::numeric_limits<float>::quiet_NaN();
  SensorCloud usable = makeTopologySensor(1, 1, 4);
  usable.points.push_back(makeTopologyPoint(0, 0.2f, 20.0f));
  usable.points.push_back(makeTopologyPoint(0, 0.3f, 0.05f));
  usable.points.push_back(makeTopologyPoint(
      0, 0.4f, std::numeric_limits<float>::infinity()));
  SensorClouds sensors;
  sensors.push_back(invalid_origin);
  sensors.push_back(usable);

  const std::vector<RaySelection> selected =
      selectStratifiedRays(sensors, 10, 0.1f, 10.0f);
  ASSERT_EQ(selected.size(), 4U);
  for (const RaySelection& selection : selected) {
    EXPECT_EQ(sensors[selection.sensor_index].sensor_id, 1U);
  }
}

TEST(StratifiedRaySamplerTest, SelectionIsStableUnderInputPermutation) {
  SensorClouds ordered;
  ordered.push_back(makeTopologySensor(0, 2, 12));
  ordered.push_back(makeTopologySensor(1, 2, 12));
  SensorClouds permuted = ordered;
  std::reverse(permuted.begin(), permuted.end());
  for (auto& sensor : permuted) {
    std::reverse(sensor.points.begin(), sensor.points.end());
  }

  const std::vector<RaySelection> ordered_selection =
      selectStratifiedRays(ordered, 19, 0.1f, 10.0f);
  const std::vector<RaySelection> permuted_selection =
      selectStratifiedRays(permuted, 19, 0.1f, 10.0f);
  ASSERT_EQ(ordered_selection.size(), permuted_selection.size());
  for (std::size_t index = 0; index < ordered_selection.size(); ++index) {
    EXPECT_EQ(selectionKey(ordered, ordered_selection[index]),
              selectionKey(permuted, permuted_selection[index]));
  }
}

TEST(StratifiedRaySamplerTest, IntegrationConsumesAtMostConfiguredRayBudget) {
  TsdfConfig config = testTsdfConfig();
  config.max_points_per_frame = 5;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeTopologySensor(0, 3, 8));
  sensors.push_back(makeTopologySensor(1, 3, 8));
  const IntegrationResult result = volume.integrateFrame(sensors, 1.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.sampled_points, 5U);
  EXPECT_EQ(result.integrated_rays, 5U);
}

TEST(SparseTsdfVolumeTest, MultipleSensorsRemainOneWindowFrame) {
  SparseTsdfVolume volume(testTsdfConfig());
  SensorClouds sensors;
  sensors.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                  Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  sensors.push_back(makeSingleRay(1, Eigen::Vector3f(2.05f, 0.05f, 0.05f),
                                  Eigen::Vector3f(3.05f, 0.05f, 0.05f)));
  const IntegrationResult result = volume.integrateFrame(sensors, 1.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.integrated_rays, 2U);
  EXPECT_EQ(volume.frameCount(), 1U);

  float value = 0.0f;
  float weight = 0.0f;
  EXPECT_TRUE(volume.voxelValue(VoxelKey{8, 0, 0}, value, weight));
  EXPECT_TRUE(volume.voxelValue(VoxelKey{28, 0, 0}, value, weight));
}

TEST(SparseTsdfVolumeTest, EvictsContributionsByTimeAndFrameCount) {
  SparseTsdfVolume volume(testTsdfConfig());
  SensorClouds sensors;
  sensors.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                  Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  ASSERT_TRUE(volume.integrateFrame(sensors, 0.0).accepted);
  ASSERT_TRUE(volume.integrateFrame(sensors, 0.5).accepted);
  const IntegrationResult latest = volume.integrateFrame(sensors, 2.0);
  ASSERT_TRUE(latest.accepted);
  EXPECT_EQ(latest.evicted_frames, 2U);
  EXPECT_EQ(volume.frameCount(), 1U);
}

TEST(SparseTsdfVolumeTest, RejectedFrameDoesNotEvictAcceptedWindow) {
  SparseTsdfVolume volume(testTsdfConfig());
  SensorClouds accepted;
  accepted.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                   Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  ASSERT_TRUE(volume.integrateFrame(accepted, 0.0).accepted);
  ASSERT_EQ(volume.frameCount(), 1U);

  SensorClouds rejected;
  rejected.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                   Eigen::Vector3f(20.05f, 0.05f, 0.05f)));
  const IntegrationResult result = volume.integrateFrame(rejected, 2.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "no_valid_rays");
  EXPECT_EQ(volume.frameCount(), 1U);
  EXPECT_DOUBLE_EQ(volume.latestStamp(), 0.0);
}

TEST(SparseTsdfVolumeTest, StopsOversizedFrameBeforeMutatingVolume) {
  TsdfConfig config = testTsdfConfig();
  config.max_voxels = 2;
  SparseTsdfVolume volume(config);
  SensorClouds sensors;
  sensors.push_back(makeSingleRay(0, Eigen::Vector3f(0.05f, 0.05f, 0.05f),
                                  Eigen::Vector3f(1.05f, 0.05f, 0.05f)));
  const IntegrationResult result = volume.integrateFrame(sensors, 1.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "frame_exceeds_voxel_capacity");
  EXPECT_EQ(volume.frameCount(), 0U);
  EXPECT_EQ(volume.voxelCount(), 0U);
}

TEST(SparseTsdfVolumeTest, ExtractsIndexedPlaneSurface) {
  SparseTsdfVolume volume(testTsdfConfig());
  SensorClouds sensors;
  for (int y = -4; y <= 4; ++y) {
    for (int z = -4; z <= 4; ++z) {
      const float y_center = static_cast<float>(y) * 0.10f + 0.05f;
      const float z_center = static_cast<float>(z) * 0.10f + 0.05f;
      sensors.push_back(makeSingleRay(
          0, Eigen::Vector3f(0.05f, y_center, z_center),
          Eigen::Vector3f(1.05f, y_center, z_center)));
    }
  }
  ASSERT_TRUE(volume.integrateFrame(sensors, 1.0).accepted);
  MeshExtractionConfig mesh_config;
  mesh_config.minimum_weight = 0.5f;
  mesh_config.max_triangles = 10000;
  const IndexedMesh mesh = volume.extractMesh(mesh_config);
  ASSERT_FALSE(mesh.vertices.empty());
  ASSERT_FALSE(mesh.triangles.empty());
  EXPECT_FALSE(mesh.triangle_limit_reached);
  for (const auto& triangle : mesh.triangles) {
    EXPECT_GE(triangle.x(), 0);
    EXPECT_GE(triangle.y(), 0);
    EXPECT_GE(triangle.z(), 0);
    EXPECT_LT(static_cast<std::size_t>(triangle.x()), mesh.vertices.size());
    EXPECT_LT(static_cast<std::size_t>(triangle.y()), mesh.vertices.size());
    EXPECT_LT(static_cast<std::size_t>(triangle.z()), mesh.vertices.size());
  }
}

TEST(RegistrationGateTest, ReturnsStableStructuredReasons) {
  RegistrationConfig config;
  config.min_points = 10;
  RegistrationQuality quality;
  quality.source_points = 9;
  quality.target_points = 20;
  EXPECT_EQ(registrationRejectReason(quality, config), "too_few_points");

  quality.source_points = 20;
  quality.target_points = 20;
  EXPECT_EQ(registrationRejectReason(quality, config), "not_converged");

  quality.converged = true;
  quality.transform_finite = true;
  quality.fitness_score = config.max_fitness_score * 2.0;
  quality.inlier_ratio = 1.0;
  EXPECT_EQ(registrationRejectReason(quality, config), "fitness_gate");

  quality.fitness_score = 0.0;
  quality.inlier_ratio = 0.0;
  EXPECT_EQ(registrationRejectReason(quality, config), "inlier_gate");

  quality.inlier_ratio = 1.0;
  EXPECT_EQ(registrationRejectReason(quality, config), "accepted");
}

TEST(FrameRegistrarTest, RejectsNonfiniteConfiguration) {
  RegistrationConfig config;
  config.max_correspondence_distance =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW(FrameRegistrar registrar(config), std::invalid_argument);
}

TEST(MotionConsistencyGateTest, RejectsFalseIdentityBootstrapByDefault) {
  RegistrationConfig config;
  config.allow_unverified_bootstrap = false;
  config.minimum_bootstrap_translation = 0.10;
  config.minimum_bootstrap_rotation_rad = 0.01;
  Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();
  candidate(0, 3) = 0.02f;
  EXPECT_EQ(motionConsistencyRejectReason(candidate, Eigen::Matrix4f::Identity(),
                                          false, config),
            "degenerate_motion");
  candidate(0, 3) = 0.30f;
  EXPECT_EQ(motionConsistencyRejectReason(candidate, Eigen::Matrix4f::Identity(),
                                          false, config),
            "degenerate_motion");
  config.allow_unverified_bootstrap = true;
  EXPECT_EQ(motionConsistencyRejectReason(candidate, Eigen::Matrix4f::Identity(),
                                          false, config),
            "accepted");
}

TEST(MotionConsistencyGateTest, RejectsPredictionInconsistentMotion) {
  RegistrationConfig config;
  config.max_prediction_translation_error = 0.20;
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  prediction(0, 3) = -0.30f;
  Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();
  candidate(0, 3) = 0.10f;
  EXPECT_EQ(motionConsistencyRejectReason(candidate, prediction, true, config),
            "motion_prediction_translation_gate");
}

TEST(FrameRegistrarTest, RecoversSmallSyntheticRigidTransform) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 7; ++y) {
      for (int z = 0; z < 4; ++z) {
        const float px = 0.17f * x + 0.013f * y * y;
        const float py = 0.19f * y + 0.007f * x * z;
        const float pz = 0.23f * z + 0.011f * x * y;
        target->points.emplace_back(px, py, pz);
      }
    }
  }
  target->width = static_cast<std::uint32_t>(target->size());
  target->height = 1;
  target->is_dense = true;

  Eigen::Matrix4f target_from_source = Eigen::Matrix4f::Identity();
  target_from_source.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(0.04f, Eigen::Vector3f(0.3f, 0.4f, 0.8f).normalized())
          .toRotationMatrix();
  target_from_source(0, 3) = -0.12f;
  target_from_source(1, 3) = 0.07f;
  target_from_source(2, 3) = -0.03f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*target, *source, target_from_source.inverse());

  RegistrationConfig config;
  config.min_points = 50;
  config.correspondence_randomness = 10;
  config.max_correspondence_distance = 0.50;
  config.max_fitness_score = 0.01;
  config.inlier_distance = 0.10;
  config.min_inlier_ratio = 0.90;
  config.max_translation = 0.50;
  FrameRegistrar registrar(config);
  const RegistrationResult result = registrar.align(source, target);
  ASSERT_TRUE(result.accepted) << result.reason << " fitness="
                               << result.quality.fitness_score << " inliers="
                               << result.quality.inlier_ratio;
  EXPECT_LT((result.target_from_source.block<3, 1>(0, 3) -
             target_from_source.block<3, 1>(0, 3))
                .norm(),
            0.04f);
  EXPECT_LT((result.target_from_source.block<3, 3>(0, 0) -
             target_from_source.block<3, 3>(0, 0))
                .norm(),
            0.08f);
}

}  // namespace
}  // namespace local_tsdf_mesh

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
