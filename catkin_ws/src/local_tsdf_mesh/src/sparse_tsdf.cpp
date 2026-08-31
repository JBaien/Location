#include "local_tsdf_mesh/sparse_tsdf.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <iterator>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <Eigen/Geometry>

#include "local_tsdf_mesh/scan_strip_support.h"

namespace local_tsdf_mesh {
namespace {

constexpr std::size_t kMaximumBandSamplesPerRay = 256U;
constexpr std::size_t kAzimuthSectorCount = 360U;
constexpr double kTwoPi = 6.28318530717958647692;

struct TimedSupportBuild {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanStripSupportResult support;
  double elapsed_ms = 0.0;
};

TimedSupportBuild buildSupportTimed(
    const SensorClouds& sensors, const ScanStripSupportConfig& config) {
  TimedSupportBuild timed;
  const auto start = std::chrono::steady_clock::now();
  try {
    timed.support = buildSupportedScanStrips(sensors, config);
  } catch (...) {
    // Support is optional augmentation. A worker-side exception must never
    // expose a partial strip or prevent an otherwise valid measured frame
    // from reaching the existing transactional commit path.
    timed.support = ScanStripSupportResult();
    timed.support.reason = "support_build_exception";
    timed.support.mesh.reason = "support_build_exception";
  }
  timed.elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start)
          .count();
  return timed;
}

TimedSupportBuild failedSupportLaunch() {
  TimedSupportBuild timed;
  timed.support.reason = "support_build_exception";
  timed.support.mesh.reason = "support_build_exception";
  return timed;
}

struct RayCandidate {
  std::size_t point_index = 0;
  std::uint16_t ring = 0;
  std::uint16_t sector = 0;
  double azimuth = 0.0;
  float range = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct SectorBucket {
  std::size_t sector = 0;
  std::size_t ray_offset = 0;
  std::size_t ray_count = 0;
  std::size_t next = 0;
};

struct RingBucket {
  std::uint16_t ring = 0;
  std::vector<SectorBucket> sectors;
  std::vector<std::size_t> sector_order;
  std::size_t sector_cursor = 0;
  std::size_t remaining = 0;
};

struct SensorBucket {
  std::uint8_t sensor_id = 0;
  std::size_t sensor_index = 0;
  std::vector<RingBucket> rings;
  std::vector<std::size_t> ring_order;
  std::size_t ring_cursor = 0;
  std::size_t remaining = 0;
};

double normalizeAzimuth(double azimuth) {
  if (!std::isfinite(azimuth)) {
    return 0.0;
  }
  azimuth = std::fmod(azimuth, kTwoPi);
  if (azimuth < 0.0) {
    azimuth += kTwoPi;
  }
  return azimuth >= kTwoPi ? 0.0 : azimuth;
}

std::size_t sectorForAzimuth(double azimuth) {
  const double scaled =
      azimuth / kTwoPi * static_cast<double>(kAzimuthSectorCount);
  return std::min(kAzimuthSectorCount - 1U,
                  static_cast<std::size_t>(std::floor(scaled)));
}

// Breadth-first interval midpoints give a progressive, deterministic ordering
// whose prefixes spread across a sorted list instead of walking adjacent
// source indices.
std::vector<std::size_t> midpointOrder(std::size_t count) {
  struct Interval {
    std::size_t begin = 0;
    std::size_t end = 0;
  };
  std::vector<std::size_t> order;
  order.reserve(count);
  if (count == 0) {
    return order;
  }
  std::deque<Interval> pending;
  pending.push_back(Interval{0U, count});
  while (!pending.empty()) {
    const Interval interval = pending.front();
    pending.pop_front();
    if (interval.begin >= interval.end) {
      continue;
    }
    const std::size_t middle = interval.begin +
                               (interval.end - interval.begin) / 2U;
    order.push_back(middle);
    if (interval.begin < middle) {
      pending.push_back(Interval{interval.begin, middle});
    }
    if (middle + 1U < interval.end) {
      pending.push_back(Interval{middle + 1U, interval.end});
    }
  }
  return order;
}

// Farthest-first sectors make even very small per-ring budgets cover distinct
// directions.  The order is independent of the final ray budget, which is
// essential for prefix/nesting monotonicity.
std::vector<std::size_t> farthestSectorOrder(
    const std::vector<SectorBucket>& sectors) {
  std::vector<std::size_t> order;
  order.reserve(sectors.size());
  if (sectors.empty()) {
    return order;
  }
  if (sectors.size() == 1U) {
    order.push_back(0U);
    return order;
  }

  // Selected sectors split the circle into independent arcs.  The best next
  // sector in each arc is the occupied sector nearest its midpoint.  A heap of
  // those arc candidates is exactly equivalent to rescanning every unselected
  // sector, but reduces a full 360-sector ring from O(K^2) to O(K log K).
  struct Gap {
    std::size_t left = 0;
    std::size_t right = 0;
    std::size_t candidate = 0;
    std::size_t distance = 0;
    std::size_t sector = 0;
  };
  struct WorseGap {
    bool operator()(const Gap& left, const Gap& right) const {
      if (left.distance != right.distance) {
        return left.distance < right.distance;
      }
      return left.sector > right.sector;
    }
  };

  const std::size_t count = sectors.size();
  std::vector<std::size_t> coordinates;
  coordinates.reserve(count + 1U);
  for (const auto& sector : sectors) {
    coordinates.push_back(sector.sector);
  }
  coordinates.push_back(sectors.front().sector + kAzimuthSectorCount);

  const auto make_gap = [&coordinates, &sectors](std::size_t left,
                                                  std::size_t right,
                                                  Gap& gap) {
    if (right <= left + 1U) {
      return false;
    }
    const std::size_t target =
        coordinates[left] + (coordinates[right] - coordinates[left]) / 2U;
    auto upper = std::lower_bound(coordinates.begin() + left + 1U,
                                  coordinates.begin() + right, target);
    std::size_t best = static_cast<std::size_t>(upper - coordinates.begin());
    if (best >= right) {
      best = right - 1U;
    }
    if (best > left + 1U) {
      const std::size_t previous = best - 1U;
      const std::size_t best_distance = std::min(
          coordinates[best] - coordinates[left],
          coordinates[right] - coordinates[best]);
      const std::size_t previous_distance = std::min(
          coordinates[previous] - coordinates[left],
          coordinates[right] - coordinates[previous]);
      if (previous_distance > best_distance ||
          (previous_distance == best_distance &&
           sectors[previous].sector < sectors[best].sector)) {
        best = previous;
      }
    }
    gap.left = left;
    gap.right = right;
    gap.candidate = best;
    gap.distance = std::min(coordinates[best] - coordinates[left],
                            coordinates[right] - coordinates[best]);
    gap.sector = sectors[best].sector;
    return true;
  };

  std::priority_queue<Gap, std::vector<Gap>, WorseGap> gaps;
  order.push_back(0U);
  Gap initial;
  if (make_gap(0U, count, initial)) {
    gaps.push(initial);
  }
  while (!gaps.empty()) {
    const Gap best = gaps.top();
    gaps.pop();
    order.push_back(best.candidate);
    Gap left;
    if (make_gap(best.left, best.candidate, left)) {
      gaps.push(left);
    }
    Gap right;
    if (make_gap(best.candidate, best.right, right)) {
      gaps.push(right);
    }
  }
  return order;
}

bool popRingRay(RingBucket& ring,
                const std::vector<RaySelection>& ordered_rays,
                RaySelection& selection) {
  if (ring.remaining == 0 || ring.sectors.empty()) {
    return false;
  }
  for (std::size_t attempt = 0; attempt < ring.sector_order.size(); ++attempt) {
    const std::size_t sector_index =
        ring.sector_order[ring.sector_cursor % ring.sector_order.size()];
    ring.sector_cursor = (ring.sector_cursor + 1U) % ring.sector_order.size();
    SectorBucket& sector = ring.sectors[sector_index];
    if (sector.next >= sector.ray_count) {
      continue;
    }
    selection = ordered_rays[sector.ray_offset + sector.next++];
    --ring.remaining;
    return true;
  }
  return false;
}

bool popSensorRay(SensorBucket& sensor,
                  const std::vector<RaySelection>& ordered_rays,
                  RaySelection& selection) {
  if (sensor.remaining == 0 || sensor.rings.empty()) {
    return false;
  }
  for (std::size_t attempt = 0; attempt < sensor.ring_order.size(); ++attempt) {
    const std::size_t ring_index =
        sensor.ring_order[sensor.ring_cursor % sensor.ring_order.size()];
    sensor.ring_cursor = (sensor.ring_cursor + 1U) % sensor.ring_order.size();
    if (!popRingRay(sensor.rings[ring_index], ordered_rays, selection)) {
      continue;
    }
    --sensor.remaining;
    return true;
  }
  return false;
}

std::size_t mixHash(std::size_t seed, std::uint32_t value) {
  seed ^= static_cast<std::size_t>(value) + 0x9e3779b9U + (seed << 6U) +
          (seed >> 2U);
  return seed;
}

bool keyLess(const VoxelKey& left, const VoxelKey& right) {
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  return left.z < right.z;
}

struct EdgeKey {
  VoxelKey first;
  VoxelKey second;

  bool operator==(const EdgeKey& other) const {
    return first == other.first && second == other.second;
  }
};

struct EdgeKeyHash {
  std::size_t operator()(const EdgeKey& edge) const {
    VoxelKeyHash hash;
    const std::size_t first = hash(edge.first);
    const std::size_t second = hash(edge.second);
    return first ^ (second + 0x9e3779b9U + (first << 6U) + (first >> 2U));
  }
};

EdgeKey makeEdgeKey(const VoxelKey& a, const VoxelKey& b) {
  return keyLess(b, a) ? EdgeKey{b, a} : EdgeKey{a, b};
}

bool finitePoint(const Eigen::Vector3f& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

}  // namespace

std::vector<RaySelection> selectStratifiedRays(
    const SensorClouds& sensors, std::size_t max_rays, float min_range,
    float max_range) {
  std::vector<RaySelection> selected_rays;
  if (max_rays == 0 || !std::isfinite(min_range) ||
      !std::isfinite(max_range) || !(min_range > 0.0f) ||
      !(max_range > min_range)) {
    return selected_rays;
  }

  std::vector<SensorBucket> sensor_buckets;
  sensor_buckets.reserve(sensors.size());
  std::vector<RaySelection> ordered_rays;
  std::size_t total_input_points = 0;
  for (const auto& sensor : sensors) {
    if (sensor.points.size() >
        std::numeric_limits<std::size_t>::max() - total_input_points) {
      total_input_points = std::numeric_limits<std::size_t>::max();
      break;
    }
    total_input_points += sensor.points.size();
  }
  ordered_rays.reserve(total_input_points);
  std::unordered_map<std::size_t, std::vector<std::size_t>> midpoint_cache;

  const auto candidate_less = [](const RayCandidate& left,
                                 const RayCandidate& right) {
    if (left.ring != right.ring) {
      return left.ring < right.ring;
    }
    if (left.sector != right.sector) {
      return left.sector < right.sector;
    }
    if (left.azimuth != right.azimuth) {
      return left.azimuth < right.azimuth;
    }
    if (left.range != right.range) {
      return left.range < right.range;
    }
    if (left.x != right.x) {
      return left.x < right.x;
    }
    if (left.y != right.y) {
      return left.y < right.y;
    }
    if (left.z != right.z) {
      return left.z < right.z;
    }
    return left.point_index < right.point_index;
  };

  for (std::size_t sensor_index = 0; sensor_index < sensors.size();
       ++sensor_index) {
    const SensorCloud& sensor = sensors[sensor_index];
    if (!finitePoint(sensor.origin)) {
      continue;
    }
    std::vector<RayCandidate> candidates;
    candidates.reserve(sensor.points.size());
    for (std::size_t point_index = 0; point_index < sensor.points.size();
         ++point_index) {
      const TsdfPoint& sample = sensor.points[point_index];
      if (!finitePoint(sample.position)) {
        continue;
      }
      const Eigen::Vector3f ray = sample.position - sensor.origin;
      const float range = ray.norm();
      if (!std::isfinite(range) || range < min_range || range > max_range) {
        continue;
      }
      double azimuth = static_cast<double>(sample.azimuth);
      if (!std::isfinite(azimuth)) {
        azimuth = std::atan2(static_cast<double>(ray.y()),
                             static_cast<double>(ray.x()));
      }
      if (azimuth < 0.0 || azimuth >= kTwoPi) {
        azimuth = normalizeAzimuth(azimuth);
      }
      RayCandidate candidate;
      candidate.point_index = point_index;
      candidate.ring = sample.ring;
      candidate.sector =
          static_cast<std::uint16_t>(sectorForAzimuth(azimuth));
      candidate.azimuth = azimuth;
      candidate.range = range;
      candidate.x = sample.position.x();
      candidate.y = sample.position.y();
      candidate.z = sample.position.z();
      candidates.push_back(candidate);
    }
    if (candidates.empty()) {
      continue;
    }
    std::sort(candidates.begin(), candidates.end(), candidate_less);

    SensorBucket sensor_bucket;
    sensor_bucket.sensor_id = sensor.sensor_id;
    sensor_bucket.sensor_index = sensor_index;
    std::size_t ring_begin = 0;
    while (ring_begin < candidates.size()) {
      const std::uint16_t ring = candidates[ring_begin].ring;
      std::size_t ring_end = ring_begin + 1U;
      while (ring_end < candidates.size() &&
             candidates[ring_end].ring == ring) {
        ++ring_end;
      }
      RingBucket ring_bucket;
      ring_bucket.ring = ring;
      std::size_t sector_begin = ring_begin;
      while (sector_begin < ring_end) {
        const std::size_t sector = candidates[sector_begin].sector;
        std::size_t sector_end = sector_begin + 1U;
        while (sector_end < ring_end &&
               candidates[sector_end].sector == sector) {
          ++sector_end;
        }
        const std::size_t sector_size = sector_end - sector_begin;
        auto cached_order = midpoint_cache.find(sector_size);
        if (cached_order == midpoint_cache.end()) {
          cached_order = midpoint_cache
                             .emplace(sector_size,
                                      midpointOrder(sector_size))
                             .first;
        }
        SectorBucket sector_bucket;
        sector_bucket.sector = sector;
        sector_bucket.ray_offset = ordered_rays.size();
        sector_bucket.ray_count = sector_size;
        for (const std::size_t local_index : cached_order->second) {
          ordered_rays.push_back(RaySelection{
              sensor_index,
              candidates[sector_begin + local_index].point_index});
        }
        ring_bucket.remaining += sector_bucket.ray_count;
        ring_bucket.sectors.push_back(std::move(sector_bucket));
        sector_begin = sector_end;
      }
      ring_bucket.sector_order = farthestSectorOrder(ring_bucket.sectors);
      sensor_bucket.remaining += ring_bucket.remaining;
      sensor_bucket.rings.push_back(std::move(ring_bucket));
      ring_begin = ring_end;
    }
    sensor_bucket.ring_order = midpointOrder(sensor_bucket.rings.size());
    sensor_buckets.push_back(std::move(sensor_bucket));
  }

  std::sort(sensor_buckets.begin(), sensor_buckets.end(),
            [](const SensorBucket& left, const SensorBucket& right) {
              if (left.sensor_id != right.sensor_id) {
                return left.sensor_id < right.sensor_id;
              }
              return left.sensor_index < right.sensor_index;
            });

  const std::size_t valid_ray_count = ordered_rays.size();
  selected_rays.reserve(std::min(max_rays, valid_ray_count));
  while (selected_rays.size() < max_rays &&
         selected_rays.size() < valid_ray_count) {
    bool progressed = false;
    for (auto& sensor : sensor_buckets) {
      if (selected_rays.size() >= max_rays) {
        break;
      }
      RaySelection selection;
      if (popSensorRay(sensor, ordered_rays, selection)) {
        selected_rays.push_back(selection);
        progressed = true;
      }
    }
    if (!progressed) {
      break;
    }
  }
  return selected_rays;
}

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const {
  std::size_t seed = 2166136261U;
  seed = mixHash(seed, static_cast<std::uint32_t>(key.x));
  seed = mixHash(seed, static_cast<std::uint32_t>(key.y));
  seed = mixHash(seed, static_cast<std::uint32_t>(key.z));
  return seed;
}

MeshAppendResult appendIndexedMeshAtomic(IndexedMesh addition,
                                         std::size_t max_total_triangles,
                                         IndexedMesh& destination) {
  MeshAppendResult result;
  if (addition.triangles.empty()) {
    result.reason = "empty_addition";
    return result;
  }
  if (addition.vertices.empty()) {
    result.reason = "invalid_addition";
    return result;
  }
  if (addition.triangle_limit_reached ||
      destination.triangles.size() > max_total_triangles ||
      addition.triangles.size() >
          max_total_triangles - destination.triangles.size()) {
    result.budget_limited = true;
    result.reason = "total_triangle_capacity";
    return result;
  }

  const std::size_t maximum_index =
      static_cast<std::size_t>(std::numeric_limits<int>::max());
  if (destination.vertices.size() > maximum_index ||
      addition.vertices.size() >
          maximum_index - destination.vertices.size() + 1U) {
    result.budget_limited = true;
    result.reason = "vertex_index_capacity";
    return result;
  }
  for (const Eigen::Vector3f& vertex : addition.vertices) {
    if (!vertex.allFinite()) {
      result.reason = "invalid_addition";
      return result;
    }
  }

  const std::size_t vertex_offset = destination.vertices.size();
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
      shifted_triangles;
  shifted_triangles.reserve(addition.triangles.size());
  for (const Eigen::Vector3i& triangle : addition.triangles) {
    if (triangle.minCoeff() < 0 ||
        static_cast<std::size_t>(triangle.maxCoeff()) >=
            addition.vertices.size() ||
        triangle.x() == triangle.y() || triangle.y() == triangle.z() ||
        triangle.x() == triangle.z()) {
      result.reason = "invalid_addition";
      return result;
    }
    shifted_triangles.emplace_back(
        static_cast<int>(vertex_offset +
                         static_cast<std::size_t>(triangle.x())),
        static_cast<int>(vertex_offset +
                         static_cast<std::size_t>(triangle.y())),
        static_cast<int>(vertex_offset +
                         static_cast<std::size_t>(triangle.z())));
  }

  destination.vertices.reserve(destination.vertices.size() +
                               addition.vertices.size());
  destination.triangles.reserve(destination.triangles.size() +
                                shifted_triangles.size());
  destination.vertices.insert(
      destination.vertices.end(),
      std::make_move_iterator(addition.vertices.begin()),
      std::make_move_iterator(addition.vertices.end()));
  destination.triangles.insert(destination.triangles.end(),
                               shifted_triangles.begin(),
                               shifted_triangles.end());
  result.applied = true;
  result.reason = "applied";
  result.appended_vertices = addition.vertices.size();
  result.appended_triangles = shifted_triangles.size();
  return result;
}

SparseTsdfVolume::SparseTsdfVolume(const TsdfConfig& config) : config_(config) {
  if (!std::isfinite(config_.voxel_size) ||
      !std::isfinite(config_.truncation_distance) ||
      !std::isfinite(config_.min_range) ||
      !std::isfinite(config_.max_range) ||
      !std::isfinite(config_.max_weight_per_voxel_per_frame) ||
      !std::isfinite(config_.window_duration_sec) ||
      !(config_.voxel_size > 0.0f) ||
      !(config_.truncation_distance >= config_.voxel_size) ||
      !(config_.min_range > 0.0f) || !(config_.max_range > config_.min_range) ||
      !(config_.max_weight_per_voxel_per_frame > 0.0f) ||
      config_.window_duration_sec < 0.0 || config_.max_window_frames == 0 ||
      config_.max_points_per_frame == 0 || config_.max_voxels == 0) {
    throw std::invalid_argument("invalid sparse TSDF configuration");
  }
  const double maximum_band_samples =
      std::ceil(4.0 * static_cast<double>(config_.truncation_distance) /
                static_cast<double>(config_.voxel_size)) +
      2.0;
  if (!std::isfinite(maximum_band_samples) ||
      maximum_band_samples >
          static_cast<double>(kMaximumBandSamplesPerRay)) {
    throw std::invalid_argument(
        "TSDF truncation band is too wide for bounded ray integration");
  }
}

IntegrationResult SparseTsdfVolume::integrateFrame(
    const SensorClouds& sensors, double stamp_sec) {
  ScanStripSupportConfig disabled_support;
  disabled_support.enabled = false;
  disabled_support.voxel_size = config_.voxel_size;
  return integrateFrame(sensors, stamp_sec, disabled_support);
}

IntegrationResult SparseTsdfVolume::integrateFrame(
    const SensorClouds& sensors, double stamp_sec,
    const ScanStripSupportConfig& requested_support_config) {
  IntegrationResult result;
  for (const auto& sensor : sensors) {
    result.input_points += sensor.points.size();
  }
  if (!std::isfinite(stamp_sec)) {
    result.reason = "invalid_stamp";
    return result;
  }
  if (latest_stamp_sec_ >= 0.0 && stamp_sec + 1e-9 < latest_stamp_sec_) {
    result.reason = "stamp_regression";
    return result;
  }
  if (sensors.empty() || result.input_points == 0) {
    result.reason = "empty_frame";
    return result;
  }

  ScanStripSupportConfig support_config;
  std::future<TimedSupportBuild> support_future;
  bool support_launch_failed = false;
  if (requested_support_config.enabled) {
    support_config = requested_support_config;
    support_config.voxel_size = config_.voxel_size;
    if (config_.parallel_support_build && config_.integrate_measured_rays) {
      try {
        support_future = std::async(
            std::launch::async,
            [&sensors, support_config]() {
              return buildSupportTimed(sensors, support_config);
            });
        result.support_build_parallel = true;
      } catch (...) {
        // Do not silently fall back to a second, unbounded latency path after
        // failing to launch the explicitly requested worker. Measured
        // integration may still succeed; support remains atomically absent.
        support_launch_failed = true;
      }
    }
  }

  ContributionMap contribution;
  std::vector<RaySelection> selected_rays;
  if (config_.integrate_measured_rays) {
    selected_rays = selectStratifiedRays(
        sensors, config_.max_points_per_frame, config_.min_range,
        config_.max_range);
    result.measured_reason =
        selected_rays.empty() ? "no_valid_rays" : "accepted";
  } else {
    result.measured_reason = "disabled";
  }
  const double sample_step =
      std::max(static_cast<double>(config_.voxel_size) * 0.5, 1e-4);

  bool measured_capacity_exceeded = false;
  for (const RaySelection& selection : selected_rays) {
    const SensorCloud& sensor = sensors[selection.sensor_index];
    const Eigen::Vector3f& point =
        sensor.points[selection.point_index].position;
    const Eigen::Vector3f ray = point - sensor.origin;
    const float range = ray.norm();
    ++result.sampled_points;
    ++result.integrated_rays;
    const Eigen::Vector3f direction = ray / range;
    const double start = std::max(
        0.0, static_cast<double>(range) - config_.truncation_distance);
    const double end =
        static_cast<double>(range) + config_.truncation_distance;
    const double sample_count_value =
        std::ceil((end - start) / sample_step) + 1.0;
    if (!std::isfinite(end) || !std::isfinite(sample_count_value) ||
        sample_count_value <= 0.0 ||
        sample_count_value >
            static_cast<double>(kMaximumBandSamplesPerRay)) {
      --result.integrated_rays;
      --result.sampled_points;
      continue;
    }
    const std::size_t sample_count =
        static_cast<std::size_t>(sample_count_value);
    VoxelKey previous_key;
    bool previous_key_valid = false;
    for (std::size_t sample_index = 0; sample_index < sample_count;
         ++sample_index) {
      const double distance = std::min(
          end, start + static_cast<double>(sample_index) * sample_step);
      const Eigen::Vector3f sample =
          sensor.origin + direction * static_cast<float>(distance);
      if (!finitePoint(sample)) {
        continue;
      }
      const VoxelKey key = keyForPoint(sample);
      if (previous_key_valid && key == previous_key) {
        continue;
      }
      previous_key = key;
      previous_key_valid = true;
      const Eigen::Vector3f center = centerForKey(key);
      const float projected_distance = (center - sensor.origin).dot(direction);
      if (projected_distance < 0.0f) {
        continue;
      }
      const float signed_distance = std::max(
          -1.0f, std::min(1.0f, (range - projected_distance) /
                                    config_.truncation_distance));
      auto found = contribution.find(key);
      if (found == contribution.end()) {
        if (contribution.size() >= config_.max_voxels) {
          measured_capacity_exceeded = true;
          break;
        }
        found = contribution.emplace(key, Accumulator{}).first;
      }
      auto& voxel = found->second;
      voxel.weighted_distance_sum += signed_distance;
      voxel.weight += 1.0;
    }
    if (measured_capacity_exceeded) {
      break;
    }
  }
  if (measured_capacity_exceeded) {
    // The measured path is optional when a separately validated direct strip
    // can still produce a useful current-frame surface. Discard the entire
    // partial measured contribution, then continue through the support
    // builder. This preserves atomic measured integration and prevents a
    // near-field capacity event from starving a valid far-field direct mesh.
    contribution.clear();
    result.sampled_points = 0U;
    result.integrated_rays = 0U;
    result.measured_reason = "frame_exceeds_voxel_capacity";
    result.measured_capacity_limited = true;
  }

  // Topology-derived rays are optional augmentation.  They are first built
  // and integrated into a separate bounded map so any malformed topology or
  // capacity event can be discarded without rejecting, clearing, or changing
  // the ordinary measured-ray contribution for this frame.
  if (requested_support_config.enabled) {
    TimedSupportBuild timed_support;
    if (result.support_build_parallel) {
      try {
        timed_support = support_future.get();
      } catch (...) {
        // buildSupportTimed catches builder exceptions. This final guard also
        // fails closed if the future shared state itself cannot deliver.
        timed_support = failedSupportLaunch();
      }
    } else if (support_launch_failed) {
      timed_support = failedSupportLaunch();
    } else {
      timed_support = buildSupportTimed(sensors, support_config);
    }
    result.support_build_ms = timed_support.elapsed_ms;
    ScanStripSupportResult support = std::move(timed_support.support);
    result.support_reason = support.reason;
    result.support_candidate_budget_limited =
        support.candidate_budget_limited;
    result.support_surface_budget_limited = support.surface_budget_limited;
    result.support_input_points = support.stats.input_points;
    result.support_topology_points = support.stats.topology_points;
    result.support_ring_pairs = support.stats.ring_pairs;
    result.support_rejected_ring_pairs = support.stats.rejected_ring_pairs;
    result.support_rejected_ring_order_pairs =
        support.stats.rejected_ring_order_pairs;
    result.support_candidate_quads = support.stats.candidate_quads;
    result.support_locally_valid_quads = support.stats.locally_valid_quads;
    result.support_strong_quads = support.stats.strong_quads;
    result.support_accepted_quads = support.stats.accepted_run_quads;
    result.support_verified_long_quads = support.stats.verified_long_quads;
    result.support_rejected_long_quads = support.stats.rejected_long_quads;
    result.support_rejected_isolated_quads =
        support.stats.rejected_isolated_quads;
    result.support_candidate_samples = support.stats.candidate_samples;
    result.support_surface_rays = support.rays.size();
    result.support_mesh_reason = support.mesh.reason;
    result.support_mesh_budget_limited = support.mesh.budget_limited;
    result.support_mesh_accepted = support.mesh.accepted;
    result.support_mesh_vertices = support.mesh.vertices.size();
    result.support_mesh_triangles = support.mesh.triangles.size();
    result.support_mesh_curve_intervals = support.mesh.curve_intervals;
    result.support_mesh_skipped_curve_intervals =
        support.mesh.skipped_curve_intervals;
    result.support_mesh_skipped_degenerate_intervals =
        support.mesh.skipped_degenerate_intervals;
    result.support_mesh_skipped_sensor_columns =
        support.mesh.skipped_sensor_columns;
    result.support_mesh_output_equivalence_input_triangles =
        support.mesh.output_equivalence_input_triangles;
    result.support_mesh_output_equivalence_removed_triangles =
        support.mesh.output_equivalence_removed_triangles;
    result.support_mesh_output_equivalence_masked_sensor_columns =
        support.mesh.output_equivalence_masked_sensor_columns;
    result.support_mesh_has_first_curve_gap =
        support.mesh.has_first_curve_gap;
    result.support_mesh_first_curve_gap_sensor_id =
        support.mesh.first_curve_gap_sensor_id;
    result.support_mesh_first_curve_gap_lower_ring =
        support.mesh.first_curve_gap_lower_ring;
    result.support_mesh_first_curve_gap_start_azimuth =
        support.mesh.first_curve_gap_start_azimuth;
    result.support_mesh_first_curve_gap_end_azimuth =
        support.mesh.first_curve_gap_end_azimuth;
    result.support_mesh_first_curve_gap_corner =
        support.mesh.first_curve_gap_corner;
    result.support_mesh.vertices = std::move(support.mesh.vertices);
    result.support_mesh.triangles = std::move(support.mesh.triangles);

    if (requested_support_config.build_surface_rays && support.accepted &&
        !support.rays.empty()) {
      const auto support_integration_start = std::chrono::steady_clock::now();
      ContributionMap support_contribution;
      const std::size_t support_voxel_capacity =
          contribution.size() >= config_.max_voxels
              ? 0U
              : config_.max_voxels - contribution.size();
      support_contribution.reserve(std::min<std::size_t>(
          support_voxel_capacity, support.rays.size() * 4U));
      const double support_band = std::min(
          static_cast<double>(config_.truncation_distance),
          static_cast<double>(config_.voxel_size) *
              static_cast<double>(support_config.integration_band_voxels));
      bool support_capacity_exceeded = support_voxel_capacity == 0U;
      for (const SupportedSurfaceRay& supported_ray : support.rays) {
        if (support_capacity_exceeded ||
            supported_ray.sensor_index >= sensors.size()) {
          break;
        }
        const SensorCloud& sensor = sensors[supported_ray.sensor_index];
        if (sensor.sensor_id != supported_ray.sensor_id) {
          continue;
        }
        const Eigen::Vector3f ray = supported_ray.position - sensor.origin;
        const float range = ray.norm();
        if (!std::isfinite(range) || range < support_config.min_range ||
            range > support_config.max_range || !(range > 0.0f)) {
          continue;
        }
        const Eigen::Vector3f direction = ray / range;
        const double start =
            std::max(0.0, static_cast<double>(range) - support_band);
        const double end = static_cast<double>(range) + support_band;
        const double sample_count_value =
            std::ceil((end - start) / sample_step) + 1.0;
        if (!std::isfinite(end) || !std::isfinite(sample_count_value) ||
            sample_count_value <= 0.0 ||
            sample_count_value >
                static_cast<double>(kMaximumBandSamplesPerRay)) {
          continue;
        }
        const double support_ray_weight =
            supported_ray.verified_long_span
                ? static_cast<double>(support_config.verified_long_ray_weight)
                : static_cast<double>(support_config.strong_ray_weight);
        ++result.support_integrated_rays;
        const std::size_t sample_count =
            static_cast<std::size_t>(sample_count_value);
        VoxelKey previous_key;
        bool previous_key_valid = false;
        for (std::size_t sample_index = 0; sample_index < sample_count;
             ++sample_index) {
          const double distance = std::min(
              end, start + static_cast<double>(sample_index) * sample_step);
          const Eigen::Vector3f sample =
              sensor.origin + direction * static_cast<float>(distance);
          if (!finitePoint(sample)) {
            continue;
          }
          const VoxelKey key = keyForPoint(sample);
          if (previous_key_valid && key == previous_key) {
            continue;
          }
          previous_key = key;
          previous_key_valid = true;
          const Eigen::Vector3f center = centerForKey(key);
          const float projected_distance =
              (center - sensor.origin).dot(direction);
          if (projected_distance < 0.0f) {
            continue;
          }
          const float signed_distance = std::max(
              -1.0f,
              std::min(1.0f, (range - projected_distance) /
                                  config_.truncation_distance));
          auto found = support_contribution.find(key);
          if (found == support_contribution.end()) {
            if (support_contribution.size() >= support_voxel_capacity) {
              support_capacity_exceeded = true;
              break;
            }
            found = support_contribution.emplace(key, Accumulator{}).first;
          }
          Accumulator& voxel = found->second;
          const double remaining_weight =
              std::max(0.0,
                       static_cast<double>(
                           support_config.maximum_weight_per_voxel) -
                           voxel.weight);
          const double added_weight =
              std::min(support_ray_weight, remaining_weight);
          voxel.weighted_distance_sum += signed_distance * added_weight;
          voxel.weight += added_weight;
        }
      }
      if (support_capacity_exceeded) {
        support_contribution.clear();
        result.support_reason = "support_voxel_capacity";
        result.support_integrated_rays = 0U;
      } else {
        result.support_contributed_voxels = support_contribution.size();
        for (const auto& item : support_contribution) {
          auto& destination = contribution[item.first];
          destination.weighted_distance_sum += item.second.weighted_distance_sum;
          destination.weight += item.second.weight;
        }
      }
      result.support_integration_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - support_integration_start)
              .count();
    }
  }

  if (contribution.empty()) {
    if (result.support_mesh_accepted &&
        !result.support_mesh.triangles.empty()) {
      // A direct indexed strip is a complete current-frame output and does
      // not need a backing TSDF contribution. Keep the volume fully
      // transactional: downstream output-equivalence/geometry validation
      // must succeed before the node commits any frame-level state.
      result.accepted = true;
      result.reason = "direct_surface_only";
      result.active_frames = frames_.size();
      result.active_voxels = voxels_.size();
      return result;
    }
    result.reason = result.measured_capacity_limited
                        ? result.measured_reason
                        : "no_valid_rays";
    result.active_frames = frames_.size();
    result.active_voxels = voxels_.size();
    return result;
  }

  for (auto& item : contribution) {
    auto& voxel = item.second;
    if (voxel.weight > config_.max_weight_per_voxel_per_frame) {
      const double scale =
          static_cast<double>(config_.max_weight_per_voxel_per_frame) /
          voxel.weight;
      voxel.weighted_distance_sum *= scale;
      voxel.weight = config_.max_weight_per_voxel_per_frame;
    }
  }
  result.contributed_voxels = contribution.size();
  if (contribution.size() > config_.max_voxels) {
    result.reason = "frame_exceeds_voxel_capacity";
    result.active_frames = frames_.size();
    result.active_voxels = voxels_.size();
    return result;
  }

  // Do not mutate the accepted window until the new frame has passed all
  // validation and produced a bounded contribution.
  result.evicted_frames += evictExpired(stamp_sec);
  while (!frames_.empty() && projectedVoxelCount(contribution) > config_.max_voxels) {
    removeOldestFrame();
    ++result.evicted_frames;
  }
  if (projectedVoxelCount(contribution) > config_.max_voxels) {
    result.reason = "voxel_capacity";
    result.active_frames = frames_.size();
    result.active_voxels = voxels_.size();
    return result;
  }
  while (frames_.size() >= config_.max_window_frames) {
    removeOldestFrame();
    ++result.evicted_frames;
  }

  for (const auto& item : contribution) {
    auto& global = voxels_[item.first];
    global.weighted_distance_sum += item.second.weighted_distance_sum;
    global.weight += item.second.weight;
  }
  FrameContribution frame;
  frame.stamp_sec = stamp_sec;
  frame.voxels = std::move(contribution);
  frames_.push_back(std::move(frame));
  latest_stamp_sec_ = stamp_sec;

  result.accepted = true;
  result.volume_updated = true;
  result.reason = "accepted";
  result.active_frames = frames_.size();
  result.active_voxels = voxels_.size();
  return result;
}

IndexedMesh SparseTsdfVolume::extractMesh(
    const MeshExtractionConfig& config) const {
  return extractMesh(config, Eigen::Vector3f::Zero());
}

IndexedMesh SparseTsdfVolume::extractMesh(
    const MeshExtractionConfig& config, const Eigen::Vector3f& focus) const {
  IndexedMesh mesh;
  if (voxels_.empty() || config.max_triangles == 0 ||
      !std::isfinite(config.minimum_weight) ||
      !std::isfinite(config.iso_level) || config.minimum_weight <= 0.0f ||
      !finitePoint(focus)) {
    return mesh;
  }

  static const std::array<std::array<int, 3>, 8> kCornerOffsets = {{{{0, 0, 0}},
                                                                    {{1, 0, 0}},
                                                                    {{0, 1, 0}},
                                                                    {{1, 1, 0}},
                                                                    {{0, 0, 1}},
                                                                    {{1, 0, 1}},
                                                                    {{0, 1, 1}},
                                                                    {{1, 1, 1}}}};
  static const std::array<std::array<int, 4>, 6> kTetrahedra = {{{{0, 1, 3, 7}},
                                                                 {{0, 3, 2, 7}},
                                                                 {{0, 2, 6, 7}},
                                                                 {{0, 6, 4, 7}},
                                                                 {{0, 4, 5, 7}},
                                                                 {{0, 5, 1, 7}}}};

  std::unordered_map<EdgeKey, std::uint32_t, EdgeKeyHash> edge_vertices;
  edge_vertices.reserve(std::min<std::size_t>(voxels_.size() * 2U,
                                               config.max_triangles * 2U));

  auto valueForKey = [this, &config](const VoxelKey& key, float& value) {
    const auto found = voxels_.find(key);
    if (found == voxels_.end() ||
        found->second.weight + 1e-9 < static_cast<double>(config.minimum_weight)) {
      return false;
    }
    value = static_cast<float>(found->second.weighted_distance_sum /
                               found->second.weight);
    return std::isfinite(value);
  };

  auto vertexForEdge =
      [this, &mesh, &edge_vertices, &config](const VoxelKey& a,
                                             const VoxelKey& b, float value_a,
                                             float value_b) -> std::uint32_t {
    const EdgeKey edge = makeEdgeKey(a, b);
    const auto existing = edge_vertices.find(edge);
    if (existing != edge_vertices.end()) {
      return existing->second;
    }
    const float denominator = value_b - value_a;
    float ratio = 0.5f;
    if (std::fabs(denominator) > 1e-8f) {
      ratio = (config.iso_level - value_a) / denominator;
    }
    ratio = std::max(0.0f, std::min(1.0f, ratio));
    const Eigen::Vector3f vertex =
        centerForKey(a) + ratio * (centerForKey(b) - centerForKey(a));
    const std::uint32_t index = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back(vertex);
    edge_vertices.emplace(edge, index);
    return index;
  };

  auto addTriangle = [&mesh](std::uint32_t a, std::uint32_t b, std::uint32_t c,
                             const Eigen::Vector3f& positive_direction) {
    if (a == b || b == c || a == c) {
      return;
    }
    const Eigen::Vector3f ab = mesh.vertices[b] - mesh.vertices[a];
    const Eigen::Vector3f ac = mesh.vertices[c] - mesh.vertices[a];
    const Eigen::Vector3f normal = ab.cross(ac);
    if (!finitePoint(normal) || normal.squaredNorm() < 1e-12f) {
      return;
    }
    if (normal.dot(positive_direction) < 0.0f) {
      std::swap(b, c);
    }
    mesh.triangles.emplace_back(static_cast<int>(a), static_cast<int>(b),
                                static_cast<int>(c));
  };

  // Capacity is a normal operating bound for the rolling volume. Traverse
  // cells deterministically from the latest sensor vicinity outwards so a cap
  // cannot select an arbitrary, flickering unordered_map prefix.
  struct OrderedBase {
    VoxelKey key;
    float distance_squared = 0.0f;
    float tsdf = 0.0f;
    bool known = false;
  };
  std::vector<OrderedBase> ordered_bases;
  ordered_bases.reserve(voxels_.size());
  for (const auto& item : voxels_) {
    OrderedBase base;
    base.key = item.first;
    base.distance_squared = (centerForKey(item.first) - focus).squaredNorm();
    if (!(item.second.weight + 1e-9 <
          static_cast<double>(config.minimum_weight))) {
      base.tsdf = static_cast<float>(item.second.weighted_distance_sum /
                                     item.second.weight);
      base.known = std::isfinite(base.tsdf);
    }
    ordered_bases.push_back(base);
  }
  std::sort(ordered_bases.begin(), ordered_bases.end(),
            [](const OrderedBase& left, const OrderedBase& right) {
              if (left.distance_squared != right.distance_squared) {
                return left.distance_squared < right.distance_squared;
              }
              return keyLess(left.key, right.key);
            });

  for (const OrderedBase& ordered_base : ordered_bases) {
    if (mesh.triangles.size() >= config.max_triangles) {
      mesh.triangle_limit_reached = true;
      break;
    }
    if (!ordered_base.known) {
      continue;
    }
    const VoxelKey& base = ordered_base.key;
    if (base.x == std::numeric_limits<std::int32_t>::max() ||
        base.y == std::numeric_limits<std::int32_t>::max() ||
        base.z == std::numeric_limits<std::int32_t>::max()) {
      continue;
    }
    std::array<VoxelKey, 8> keys;
    std::array<float, 8> values;
    std::array<bool, 8> known;
    known.fill(false);
    keys[0] = base;
    values[0] = ordered_base.tsdf;
    known[0] = true;

    // All six tetrahedra share the diagonal from corner 0 to corner 7. If
    // corner 7 is unavailable, none of this cell's tetrahedra can contribute.
    keys[7] = VoxelKey{
        static_cast<std::int32_t>(base.x + kCornerOffsets[7][0]),
        static_cast<std::int32_t>(base.y + kCornerOffsets[7][1]),
        static_cast<std::int32_t>(base.z + kCornerOffsets[7][2])};
    known[7] = valueForKey(keys[7], values[7]);
    if (!known[7]) {
      continue;
    }

    for (std::size_t corner = 1; corner < 7; ++corner) {
      keys[corner] = VoxelKey{
          static_cast<std::int32_t>(base.x + kCornerOffsets[corner][0]),
          static_cast<std::int32_t>(base.y + kCornerOffsets[corner][1]),
          static_cast<std::int32_t>(base.z + kCornerOffsets[corner][2])};
      known[corner] = valueForKey(keys[corner], values[corner]);
    }

    for (const auto& tetrahedron : kTetrahedra) {
      if (mesh.triangles.size() >= config.max_triangles) {
        mesh.triangle_limit_reached = true;
        break;
      }
      std::array<int, 4> inside;
      std::array<int, 4> outside;
      int inside_count = 0;
      int outside_count = 0;
      bool tetrahedron_known = true;
      for (const int corner : tetrahedron) {
        if (!known[static_cast<std::size_t>(corner)]) {
          tetrahedron_known = false;
          break;
        }
        if (values[static_cast<std::size_t>(corner)] < config.iso_level) {
          inside[static_cast<std::size_t>(inside_count++)] = corner;
        } else {
          outside[static_cast<std::size_t>(outside_count++)] = corner;
        }
      }
      if (!tetrahedron_known || inside_count == 0 || outside_count == 0) {
        continue;
      }

      Eigen::Vector3f negative_center = Eigen::Vector3f::Zero();
      Eigen::Vector3f positive_center = Eigen::Vector3f::Zero();
      for (int i = 0; i < inside_count; ++i) {
        negative_center += centerForKey(keys[static_cast<std::size_t>(inside[i])]);
      }
      for (int i = 0; i < outside_count; ++i) {
        positive_center += centerForKey(keys[static_cast<std::size_t>(outside[i])]);
      }
      negative_center /= static_cast<float>(inside_count);
      positive_center /= static_cast<float>(outside_count);
      const Eigen::Vector3f positive_direction = positive_center - negative_center;

      auto edgeVertex = [&](int a, int b) {
        return vertexForEdge(keys[static_cast<std::size_t>(a)],
                             keys[static_cast<std::size_t>(b)],
                             values[static_cast<std::size_t>(a)],
                             values[static_cast<std::size_t>(b)]);
      };

      if (inside_count == 1) {
        const std::uint32_t a = edgeVertex(inside[0], outside[0]);
        const std::uint32_t b = edgeVertex(inside[0], outside[1]);
        const std::uint32_t c = edgeVertex(inside[0], outside[2]);
        addTriangle(a, b, c, positive_direction);
      } else if (inside_count == 3) {
        const std::uint32_t a = edgeVertex(outside[0], inside[0]);
        const std::uint32_t b = edgeVertex(outside[0], inside[1]);
        const std::uint32_t c = edgeVertex(outside[0], inside[2]);
        addTriangle(a, c, b, positive_direction);
      } else {
        const std::uint32_t a = edgeVertex(inside[0], outside[0]);
        const std::uint32_t b = edgeVertex(inside[0], outside[1]);
        const std::uint32_t c = edgeVertex(inside[1], outside[0]);
        const std::uint32_t d = edgeVertex(inside[1], outside[1]);
        addTriangle(a, b, d, positive_direction);
        if (mesh.triangles.size() < config.max_triangles) {
          addTriangle(a, d, c, positive_direction);
        } else {
          mesh.triangle_limit_reached = true;
        }
      }
    }
  }
  return mesh;
}

void SparseTsdfVolume::clear() {
  voxels_.clear();
  frames_.clear();
  latest_stamp_sec_ = -1.0;
}

std::size_t SparseTsdfVolume::voxelCount() const { return voxels_.size(); }

std::size_t SparseTsdfVolume::frameCount() const { return frames_.size(); }

double SparseTsdfVolume::latestStamp() const { return latest_stamp_sec_; }

bool SparseTsdfVolume::voxelValue(const VoxelKey& key, float& distance,
                                  float& weight) const {
  const auto found = voxels_.find(key);
  if (found == voxels_.end() || found->second.weight <= 1e-9) {
    return false;
  }
  distance = static_cast<float>(found->second.weighted_distance_sum /
                                found->second.weight);
  weight = static_cast<float>(found->second.weight);
  return std::isfinite(distance) && std::isfinite(weight);
}

VoxelKey SparseTsdfVolume::keyForPoint(const Eigen::Vector3f& point) const {
  const double inverse_size = 1.0 / static_cast<double>(config_.voxel_size);
  auto quantize = [inverse_size](float value) {
    const double scaled = std::floor(static_cast<double>(value) * inverse_size);
    const double minimum =
        static_cast<double>(std::numeric_limits<std::int32_t>::min());
    const double maximum =
        static_cast<double>(std::numeric_limits<std::int32_t>::max());
    return static_cast<std::int32_t>(std::max(minimum, std::min(maximum, scaled)));
  };
  return VoxelKey{quantize(point.x()), quantize(point.y()), quantize(point.z())};
}

Eigen::Vector3f SparseTsdfVolume::centerForKey(const VoxelKey& key) const {
  const float half = 0.5f * config_.voxel_size;
  return Eigen::Vector3f(static_cast<float>(key.x) * config_.voxel_size + half,
                         static_cast<float>(key.y) * config_.voxel_size + half,
                         static_cast<float>(key.z) * config_.voxel_size + half);
}

std::size_t SparseTsdfVolume::evictExpired(double stamp_sec) {
  std::size_t evicted = 0;
  if (config_.window_duration_sec <= 0.0) {
    return evicted;
  }
  while (!frames_.empty() &&
         frames_.front().stamp_sec + config_.window_duration_sec < stamp_sec) {
    removeOldestFrame();
    ++evicted;
  }
  return evicted;
}

void SparseTsdfVolume::removeOldestFrame() {
  if (frames_.empty()) {
    return;
  }
  for (const auto& item : frames_.front().voxels) {
    const auto global = voxels_.find(item.first);
    if (global == voxels_.end()) {
      continue;
    }
    global->second.weighted_distance_sum -= item.second.weighted_distance_sum;
    global->second.weight -= item.second.weight;
    if (global->second.weight <= 1e-6) {
      voxels_.erase(global);
    }
  }
  frames_.pop_front();
}

std::size_t SparseTsdfVolume::projectedVoxelCount(
    const ContributionMap& contribution) const {
  std::size_t projected = voxels_.size();
  for (const auto& item : contribution) {
    if (voxels_.find(item.first) == voxels_.end()) {
      ++projected;
    }
  }
  return projected;
}

}  // namespace local_tsdf_mesh
