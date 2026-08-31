#include "local_tsdf_mesh/scan_strip_support.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "local_tsdf_mesh/mesh_cleanup.h"

namespace local_tsdf_mesh {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDuplicateAzimuthEpsilon = 1e-7;
constexpr double kMaximumMedianGapRad = kPi / 180.0;
constexpr double kSupportBinWidthRad = 0.5 * kPi / 180.0;
constexpr int kSupportBinCount = 720;

double radians(float degrees) {
  return static_cast<double>(degrees) * kPi / 180.0;
}

double normalizeAzimuth(double angle) {
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  angle = std::fmod(angle, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle >= kTwoPi ? 0.0 : angle;
}

double forwardAngle(double from, double to) {
  double difference = normalizeAzimuth(to) - normalizeAzimuth(from);
  if (difference < 0.0) {
    difference += kTwoPi;
  }
  return difference;
}

double circularDistance(double left, double right) {
  const double direct = std::fabs(normalizeAzimuth(left) -
                                  normalizeAzimuth(right));
  return std::min(direct, kTwoPi - direct);
}

double circularMean(double left, double right) {
  const double x = std::cos(left) + std::cos(right);
  const double y = std::sin(left) + std::sin(right);
  if (std::fabs(x) + std::fabs(y) < 1e-12) {
    return normalizeAzimuth(left);
  }
  return normalizeAzimuth(std::atan2(y, x));
}

bool finitePoint(const Eigen::Vector3f& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

float lengthRatio(float left, float right) {
  const float minimum = std::min(left, right);
  const float maximum = std::max(left, right);
  if (!(minimum > 1e-6f) || !std::isfinite(maximum)) {
    return std::numeric_limits<float>::infinity();
  }
  return maximum / minimum;
}

Eigen::Vector3f unitVector(const Eigen::Vector3f& vector) {
  const float norm = vector.norm();
  if (!std::isfinite(norm) || norm <= 1e-6f) {
    return Eigen::Vector3f::Zero();
  }
  return vector / norm;
}

struct TopologySample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t point_index = 0;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  float range = 0.0f;
  double azimuth = 0.0;
};

using TopologySamples =
    std::vector<TopologySample, Eigen::aligned_allocator<TopologySample>>;

struct RingScan {
  std::uint16_t ring = 0;
  TopologySamples samples;
  double median_step = std::numeric_limits<double>::quiet_NaN();
};

struct MatchedRung {
  std::size_t lower_index = 0;
  std::size_t upper_index = 0;
  double azimuth = 0.0;
  double unwrapped_azimuth = 0.0;
};

struct QuadCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint16_t lower_ring = 0;
  Eigen::Vector3f lower_start = Eigen::Vector3f::Zero();
  Eigen::Vector3f lower_end = Eigen::Vector3f::Zero();
  Eigen::Vector3f upper_start = Eigen::Vector3f::Zero();
  Eigen::Vector3f upper_end = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  Eigen::Vector3f cross_direction = Eigen::Vector3f::Zero();
  double start_azimuth = 0.0;
  double end_azimuth = 0.0;
  double midpoint_azimuth = 0.0;
  double azimuth_width = 0.0;
  double lower_start_azimuth = 0.0;
  double lower_end_azimuth = 0.0;
  double upper_start_azimuth = 0.0;
  double upper_end_azimuth = 0.0;
  float maximum_along_edge = 0.0f;
  float maximum_cross_edge = 0.0f;
  bool locally_valid = false;
  bool strong = false;
  bool verified_long = false;
  bool final_valid = false;
};

using QuadCandidates =
    std::vector<QuadCandidate, Eigen::aligned_allocator<QuadCandidate>>;

struct PairStrip {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint16_t lower_ring = 0;
  Eigen::Vector3f ring_step_direction = Eigen::Vector3f::Zero();
  bool ring_order_valid = false;
  QuadCandidates quads;
  std::map<int, std::vector<std::size_t>> azimuth_bins;
};

struct RasterWork {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadCandidate quad;
  bool include_end = false;
};

using RasterWorks =
    std::vector<RasterWork, Eigen::aligned_allocator<RasterWork>>;

struct SurfaceCellKey {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;

  bool operator<(const SurfaceCellKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct SurfaceCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  std::uint16_t lower_ring = 0;
  float azimuth = 0.0f;
  bool verified_long_span = false;
  float center_distance_squared = std::numeric_limits<float>::infinity();
};

using SurfaceCellMap = std::map<SurfaceCellKey, SurfaceCandidate>;

bool validConfiguration(const ScanStripSupportConfig& config) {
  const bool finite =
      std::isfinite(config.voxel_size) && std::isfinite(config.min_range) &&
      std::isfinite(config.max_range) &&
      std::isfinite(config.surface_spacing_voxels) &&
      std::isfinite(config.integration_band_voxels) &&
      std::isfinite(config.strong_ray_weight) &&
      std::isfinite(config.verified_long_ray_weight) &&
      std::isfinite(config.maximum_weight_per_voxel) &&
      std::isfinite(config.maximum_mesh_edge_voxels) &&
      std::isfinite(config.mesh_weld_tolerance_voxels) &&
      std::isfinite(config.minimum_ring_angle_deg) &&
      std::isfinite(config.maximum_ring_angle_deg) &&
      std::isfinite(config.maximum_ring_order_direction_angle_deg) &&
      std::isfinite(config.minimum_match_angle_deg) &&
      std::isfinite(config.maximum_match_angle_deg) &&
      std::isfinite(config.match_step_factor) &&
      std::isfinite(config.minimum_gap_angle_deg) &&
      std::isfinite(config.maximum_gap_angle_deg) &&
      std::isfinite(config.gap_step_factor) &&
      std::isfinite(config.maximum_along_log_range_delta) &&
      std::isfinite(config.maximum_along_edge_base_m) &&
      std::isfinite(config.maximum_along_edge_range_ratio) &&
      std::isfinite(config.strong_cross_log_range_delta) &&
      std::isfinite(config.strong_cross_edge_base_m) &&
      std::isfinite(config.strong_cross_edge_range_ratio) &&
      std::isfinite(config.maximum_cross_log_range_delta) &&
      std::isfinite(config.maximum_cross_edge_base_m) &&
      std::isfinite(config.maximum_cross_edge_range_ratio) &&
      std::isfinite(config.maximum_cross_edge_absolute_m) &&
      std::isfinite(config.maximum_quad_normal_angle_deg) &&
      std::isfinite(config.maximum_neighbor_normal_angle_deg) &&
      std::isfinite(config.maximum_opposite_edge_length_ratio) &&
      std::isfinite(config.maximum_planarity_base_m) &&
      std::isfinite(config.maximum_planarity_edge_ratio) &&
      std::isfinite(config.maximum_neighbor_plane_edge_ratio) &&
      std::isfinite(config.maximum_run_cross_edge_ratio);
  return finite && config.voxel_size > 0.0f && config.min_range > 0.0f &&
         config.max_range > config.min_range &&
         config.max_surface_cells > 0U && config.max_candidate_samples > 0U &&
         config.surface_spacing_voxels > 0.0f &&
         config.integration_band_voxels >= 1.0f &&
         config.strong_ray_weight > 0.0f &&
         config.verified_long_ray_weight > 0.0f &&
         config.verified_long_ray_weight <= config.strong_ray_weight &&
         config.maximum_weight_per_voxel > 0.0f &&
         config.strong_ray_weight <= config.maximum_weight_per_voxel &&
         config.max_mesh_vertices > 0U && config.max_mesh_triangles > 0U &&
         config.max_mesh_vertices <=
             static_cast<std::size_t>(std::numeric_limits<int>::max()) &&
         config.maximum_mesh_edge_voxels > 0.0f &&
         config.mesh_weld_tolerance_voxels > 0.0f &&
         config.mesh_weld_tolerance_voxels <
             config.maximum_mesh_edge_voxels &&
         config.mesh_weld_tolerance_voxels <=
             0.01f * config.maximum_mesh_edge_voxels &&
         std::isfinite(static_cast<double>(config.voxel_size) *
                       static_cast<double>(config.surface_spacing_voxels)) &&
         static_cast<double>(config.voxel_size) *
                 static_cast<double>(config.surface_spacing_voxels) >
             0.0 &&
         std::isfinite(static_cast<double>(config.voxel_size) *
                       static_cast<double>(config.integration_band_voxels)) &&
         config.minimum_ring_angle_deg > 0.0f &&
         config.maximum_ring_angle_deg >= config.minimum_ring_angle_deg &&
         config.maximum_ring_order_direction_angle_deg > 0.0f &&
         config.maximum_ring_order_direction_angle_deg < 90.0f &&
         config.minimum_match_angle_deg > 0.0f &&
         config.maximum_match_angle_deg >= config.minimum_match_angle_deg &&
         config.match_step_factor > 0.0f &&
         config.minimum_gap_angle_deg > 0.0f &&
         config.maximum_gap_angle_deg >= config.minimum_gap_angle_deg &&
         config.gap_step_factor > 0.0f &&
         config.maximum_along_log_range_delta >= 0.0f &&
         config.maximum_along_edge_base_m > 0.0f &&
         config.maximum_along_edge_range_ratio > 0.0f &&
         config.strong_cross_log_range_delta >= 0.0f &&
         config.maximum_cross_log_range_delta >=
             config.strong_cross_log_range_delta &&
         config.strong_cross_edge_base_m > 0.0f &&
         config.strong_cross_edge_range_ratio > 0.0f &&
         config.maximum_cross_edge_base_m >=
             config.strong_cross_edge_base_m &&
         config.maximum_cross_edge_range_ratio >=
             config.strong_cross_edge_range_ratio &&
         config.maximum_cross_edge_absolute_m > 0.0f &&
         config.maximum_quad_normal_angle_deg > 0.0f &&
         config.maximum_quad_normal_angle_deg < 90.0f &&
         config.maximum_neighbor_normal_angle_deg > 0.0f &&
         config.maximum_neighbor_normal_angle_deg < 90.0f &&
         config.maximum_opposite_edge_length_ratio >= 1.0f &&
         config.maximum_planarity_base_m > 0.0f &&
         config.maximum_planarity_edge_ratio >= 0.0f &&
         config.maximum_neighbor_plane_edge_ratio >= 0.0f &&
         config.maximum_run_cross_edge_ratio >= 1.0f &&
         config.minimum_run_quads > 0U;
}

double estimateMedianStep(const TopologySamples& samples) {
  if (samples.size() < 2U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> gaps;
  gaps.reserve(samples.size());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const std::size_t next = (index + 1U) % samples.size();
    const double gap = forwardAngle(samples[index].azimuth,
                                    samples[next].azimuth);
    if (gap > kDuplicateAzimuthEpsilon && gap <= kMaximumMedianGapRad) {
      gaps.push_back(gap);
    }
  }
  return median(std::move(gaps));
}

std::map<std::uint16_t, RingScan> buildRingScans(
    const SensorCloud& sensor, const ScanStripSupportConfig& config,
    ScanStripSupportStats& stats) {
  std::map<std::uint16_t, TopologySamples> grouped;
  stats.input_points += sensor.points.size();
  if (!finitePoint(sensor.origin)) {
    return {};
  }
  for (std::size_t point_index = 0; point_index < sensor.points.size();
       ++point_index) {
    const TsdfPoint& point = sensor.points[point_index];
    if (!point.source_topology_valid || !finitePoint(point.position) ||
        !std::isfinite(point.azimuth)) {
      continue;
    }
    const float range = (point.position - sensor.origin).norm();
    if (!std::isfinite(range) || range < config.min_range ||
        range > config.max_range) {
      continue;
    }
    TopologySample sample;
    sample.point_index = point_index;
    sample.position = point.position;
    sample.range = range;
    sample.azimuth = normalizeAzimuth(point.azimuth);
    grouped[point.ring].push_back(sample);
    ++stats.topology_points;
  }

  std::map<std::uint16_t, RingScan> scans;
  for (auto& item : grouped) {
    TopologySamples& samples = item.second;
    std::sort(samples.begin(), samples.end(),
              [](const TopologySample& left, const TopologySample& right) {
                if (left.azimuth != right.azimuth) {
                  return left.azimuth < right.azimuth;
                }
                if (left.range != right.range) {
                  return left.range < right.range;
                }
                if (left.position.x() != right.position.x()) {
                  return left.position.x() < right.position.x();
                }
                if (left.position.y() != right.position.y()) {
                  return left.position.y() < right.position.y();
                }
                if (left.position.z() != right.position.z()) {
                  return left.position.z() < right.position.z();
                }
                return left.point_index < right.point_index;
              });
    TopologySamples unique;
    unique.reserve(samples.size());
    for (const TopologySample& sample : samples) {
      if (!unique.empty() &&
          circularDistance(unique.back().azimuth, sample.azimuth) <=
              kDuplicateAzimuthEpsilon) {
        continue;
      }
      unique.push_back(sample);
    }
    if (unique.size() < 3U) {
      continue;
    }
    RingScan scan;
    scan.ring = item.first;
    scan.samples = std::move(unique);
    scan.median_step = estimateMedianStep(scan.samples);
    if (std::isfinite(scan.median_step) && scan.median_step > 0.0) {
      scans.emplace(scan.ring, std::move(scan));
    }
  }
  return scans;
}

std::size_t nearestSample(const TopologySamples& samples, double azimuth) {
  const auto found = std::lower_bound(
      samples.begin(), samples.end(), azimuth,
      [](const TopologySample& sample, double value) {
        return sample.azimuth < value;
      });
  const std::size_t upper =
      found == samples.end()
          ? 0U
          : static_cast<std::size_t>(found - samples.begin());
  const std::size_t lower =
      upper == 0U ? samples.size() - 1U : upper - 1U;
  const double lower_distance =
      circularDistance(samples[lower].azimuth, azimuth);
  const double upper_distance =
      circularDistance(samples[upper].azimuth, azimuth);
  return lower_distance <= upper_distance ? lower : upper;
}

std::vector<MatchedRung> matchRungs(const RingScan& lower,
                                    const RingScan& upper,
                                    const ScanStripSupportConfig& config) {
  const double match_limit = std::max(
      radians(config.minimum_match_angle_deg),
      std::min(radians(config.maximum_match_angle_deg),
               static_cast<double>(config.match_step_factor) *
                   std::max(lower.median_step, upper.median_step)));
  std::vector<MatchedRung> rungs;
  rungs.reserve(std::min(lower.samples.size(), upper.samples.size()));
  for (std::size_t lower_index = 0; lower_index < lower.samples.size();
       ++lower_index) {
    const std::size_t upper_index =
        nearestSample(upper.samples, lower.samples[lower_index].azimuth);
    if (nearestSample(lower.samples, upper.samples[upper_index].azimuth) !=
        lower_index) {
      continue;
    }
    if (circularDistance(lower.samples[lower_index].azimuth,
                         upper.samples[upper_index].azimuth) > match_limit) {
      continue;
    }
    MatchedRung rung;
    rung.lower_index = lower_index;
    rung.upper_index = upper_index;
    rung.azimuth = circularMean(lower.samples[lower_index].azimuth,
                                upper.samples[upper_index].azimuth);
    rungs.push_back(rung);
  }
  std::sort(rungs.begin(), rungs.end(),
            [](const MatchedRung& left, const MatchedRung& right) {
              if (left.azimuth != right.azimuth) {
                return left.azimuth < right.azimuth;
              }
              return std::tie(left.lower_index, left.upper_index) <
                     std::tie(right.lower_index, right.upper_index);
            });
  if (rungs.size() < 2U) {
    return rungs;
  }

  // Find out whether this is a complete circle or a partial-FOV scan.  Only a
  // genuinely unsupported interval is allowed to become an open seam.
  std::size_t largest_gap_index = 0U;
  double largest_gap = -1.0;
  for (std::size_t index = 0; index < rungs.size(); ++index) {
    const std::size_t next = (index + 1U) % rungs.size();
    const double gap = forwardAngle(rungs[index].azimuth,
                                    rungs[next].azimuth);
    if (gap > largest_gap) {
      largest_gap = gap;
      largest_gap_index = index;
    }
  }
  const double continuous_gap_limit = std::max(
      radians(config.minimum_gap_angle_deg),
      std::min(radians(config.maximum_gap_angle_deg),
               static_cast<double>(config.gap_step_factor) *
                   std::max(lower.median_step, upper.median_step)));
  if (largest_gap <= continuous_gap_limit) {
    // Preserve a genuinely complete circular scan.  Appending the first
    // matched column once at +2pi makes evaluateQuad produce last->first;
    // surface-cell deduplication later removes the repeated endpoint.
    for (std::size_t index = 0; index < rungs.size(); ++index) {
      if (index == 0U) {
        rungs[index].unwrapped_azimuth = rungs[index].azimuth;
      } else {
        rungs[index].unwrapped_azimuth =
            rungs[index - 1U].unwrapped_azimuth +
            forwardAngle(rungs[index - 1U].azimuth, rungs[index].azimuth);
      }
    }
    MatchedRung closure = rungs.front();
    closure.unwrapped_azimuth =
        rungs.back().unwrapped_azimuth +
        forwardAngle(rungs.back().azimuth, rungs.front().azimuth);
    rungs.push_back(closure);
    return rungs;
  }
  std::vector<MatchedRung> ordered;
  ordered.reserve(rungs.size());
  const std::size_t start = (largest_gap_index + 1U) % rungs.size();
  for (std::size_t offset = 0; offset < rungs.size(); ++offset) {
    ordered.push_back(rungs[(start + offset) % rungs.size()]);
    if (offset == 0U) {
      ordered.back().unwrapped_azimuth = ordered.back().azimuth;
    } else {
      ordered.back().unwrapped_azimuth =
          ordered[offset - 1U].unwrapped_azimuth +
          forwardAngle(ordered[offset - 1U].azimuth,
                       ordered.back().azimuth);
    }
  }
  return ordered;
}

double medianRingAngle(const RingScan& lower, const RingScan& upper,
                       const std::vector<MatchedRung>& rungs,
                       const Eigen::Vector3f& origin) {
  std::vector<double> angles;
  angles.reserve(rungs.size());
  for (const MatchedRung& rung : rungs) {
    const Eigen::Vector3f lower_ray = unitVector(
        lower.samples[rung.lower_index].position - origin);
    const Eigen::Vector3f upper_ray = unitVector(
        upper.samples[rung.upper_index].position - origin);
    if (lower_ray.squaredNorm() <= 0.0f || upper_ray.squaredNorm() <= 0.0f) {
      continue;
    }
    const double cosine = std::max(
        -1.0, std::min(1.0, static_cast<double>(lower_ray.dot(upper_ray))));
    angles.push_back(std::acos(cosine));
  }
  return median(std::move(angles));
}

Eigen::Vector3f ringStepDirection(const RingScan& lower,
                                  const RingScan& upper,
                                  const std::vector<MatchedRung>& rungs,
                                  const Eigen::Vector3f& origin) {
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  std::size_t count = 0U;
  for (const MatchedRung& rung : rungs) {
    const Eigen::Vector3f lower_ray = unitVector(
        lower.samples[rung.lower_index].position - origin);
    const Eigen::Vector3f upper_ray = unitVector(
        upper.samples[rung.upper_index].position - origin);
    const Eigen::Vector3f step = unitVector(upper_ray - lower_ray);
    if (step.squaredNorm() > 0.0f) {
      sum += step;
      ++count;
    }
  }
  return count == 0U ? Eigen::Vector3f::Zero() : unitVector(sum);
}

bool evaluateQuadGeometry(QuadCandidate& candidate,
                          const std::array<float, 4>& ranges,
                          const ScanStripSupportConfig& config) {
  candidate.locally_valid = false;
  candidate.strong = false;
  candidate.normal = Eigen::Vector3f::Zero();
  candidate.cross_direction = Eigen::Vector3f::Zero();
  for (const float range : ranges) {
    if (!std::isfinite(range) || !(range > 0.0f)) {
      return false;
    }
  }
  const Eigen::Vector3f lower_along =
      candidate.lower_end - candidate.lower_start;
  const Eigen::Vector3f upper_along =
      candidate.upper_end - candidate.upper_start;
  const Eigen::Vector3f start_cross =
      candidate.upper_start - candidate.lower_start;
  const Eigen::Vector3f end_cross =
      candidate.upper_end - candidate.lower_end;
  const float lower_along_length = lower_along.norm();
  const float upper_along_length = upper_along.norm();
  const float start_cross_length = start_cross.norm();
  const float end_cross_length = end_cross.norm();
  if (!(lower_along_length > 1e-5f) || !(upper_along_length > 1e-5f) ||
      !(start_cross_length > 1e-5f) || !(end_cross_length > 1e-5f)) {
    return false;
  }
  candidate.maximum_along_edge =
      std::max(lower_along_length, upper_along_length);
  candidate.maximum_cross_edge =
      std::max(start_cross_length, end_cross_length);
  const float minimum_range = std::min(
      std::min(ranges[0], ranges[1]), std::min(ranges[2], ranges[3]));

  const float along_log_delta = std::max(
      std::fabs(std::log(ranges[1] / ranges[0])),
      std::fabs(std::log(ranges[3] / ranges[2])));
  const float along_edge_limit = std::max(
      config.maximum_along_edge_base_m,
      config.maximum_along_edge_range_ratio * minimum_range);
  if (along_log_delta > config.maximum_along_log_range_delta ||
      candidate.maximum_along_edge > along_edge_limit) {
    return false;
  }

  const float cross_log_delta = std::max(
      std::fabs(std::log(ranges[2] / ranges[0])),
      std::fabs(std::log(ranges[3] / ranges[1])));
  const float hard_cross_limit = std::min(
      config.maximum_cross_edge_absolute_m,
      std::max(config.maximum_cross_edge_base_m,
               config.maximum_cross_edge_range_ratio * minimum_range));
  if (cross_log_delta > config.maximum_cross_log_range_delta ||
      candidate.maximum_cross_edge > hard_cross_limit) {
    return false;
  }

  const float quad_cosine = static_cast<float>(
      std::cos(radians(config.maximum_quad_normal_angle_deg)));
  if (unitVector(lower_along).dot(unitVector(upper_along)) < quad_cosine ||
      unitVector(start_cross).dot(unitVector(end_cross)) < quad_cosine ||
      lengthRatio(lower_along_length, upper_along_length) >
          config.maximum_opposite_edge_length_ratio ||
      lengthRatio(start_cross_length, end_cross_length) >
          config.maximum_opposite_edge_length_ratio) {
    return false;
  }

  const Eigen::Vector3f diagonal =
      candidate.upper_end - candidate.lower_start;
  const Eigen::Vector3f first_normal =
      unitVector(lower_along.cross(diagonal));
  const Eigen::Vector3f second_normal =
      unitVector(diagonal.cross(start_cross));
  if (first_normal.squaredNorm() <= 0.0f ||
      second_normal.squaredNorm() <= 0.0f ||
      first_normal.dot(second_normal) < quad_cosine) {
    return false;
  }
  candidate.normal = unitVector(first_normal + second_normal);
  candidate.cross_direction = unitVector(start_cross + end_cross);
  if (candidate.normal.squaredNorm() <= 0.0f ||
      candidate.cross_direction.squaredNorm() <= 0.0f) {
    return false;
  }
  const Eigen::Vector3f start_plane_normal =
      unitVector(lower_along.cross(start_cross));
  const Eigen::Vector3f end_plane_normal =
      unitVector(upper_along.cross(end_cross));
  const float planarity_error = std::max(
      std::fabs((candidate.upper_end - candidate.lower_start)
                    .dot(start_plane_normal)),
      std::fabs((candidate.lower_start - candidate.upper_end)
                    .dot(end_plane_normal)));
  const float planarity_limit = std::max(
      config.maximum_planarity_base_m,
      config.maximum_planarity_edge_ratio * candidate.maximum_cross_edge);
  if (!std::isfinite(planarity_error) || planarity_error > planarity_limit) {
    return false;
  }

  candidate.locally_valid = true;
  const float strong_cross_limit = std::max(
      config.strong_cross_edge_base_m,
      config.strong_cross_edge_range_ratio * minimum_range);
  candidate.strong =
      cross_log_delta <= config.strong_cross_log_range_delta &&
      candidate.maximum_cross_edge <= strong_cross_limit;
  return true;
}

QuadCandidate evaluateQuad(const RingScan& lower, const RingScan& upper,
                           const MatchedRung& start,
                           const MatchedRung& end,
                           const ScanStripSupportConfig& config) {
  QuadCandidate candidate;
  candidate.lower_ring = lower.ring;
  const TopologySample& lower_start = lower.samples[start.lower_index];
  const TopologySample& lower_end = lower.samples[end.lower_index];
  const TopologySample& upper_start = upper.samples[start.upper_index];
  const TopologySample& upper_end = upper.samples[end.upper_index];
  candidate.lower_start = lower_start.position;
  candidate.lower_end = lower_end.position;
  candidate.upper_start = upper_start.position;
  candidate.upper_end = upper_end.position;
  candidate.start_azimuth = start.azimuth;
  candidate.end_azimuth = end.azimuth;
  candidate.azimuth_width = end.unwrapped_azimuth - start.unwrapped_azimuth;
  candidate.lower_start_azimuth = lower_start.azimuth;
  candidate.lower_end_azimuth = lower_end.azimuth;
  candidate.upper_start_azimuth = upper_start.azimuth;
  candidate.upper_end_azimuth = upper_end.azimuth;
  candidate.midpoint_azimuth = normalizeAzimuth(
      start.azimuth + 0.5 * candidate.azimuth_width);

  const double gap_limit = std::max(
      radians(config.minimum_gap_angle_deg),
      std::min(radians(config.maximum_gap_angle_deg),
               static_cast<double>(config.gap_step_factor) *
                   std::max(lower.median_step, upper.median_step)));
  const double lower_gap =
      forwardAngle(lower_start.azimuth, lower_end.azimuth);
  const double upper_gap =
      forwardAngle(upper_start.azimuth, upper_end.azimuth);
  if (!(candidate.azimuth_width > 0.0) || lower_gap > gap_limit ||
      upper_gap > gap_limit) {
    return candidate;
  }

  evaluateQuadGeometry(
      candidate,
      {{lower_start.range, lower_end.range, upper_start.range,
        upper_end.range}},
      config);
  return candidate;
}

int azimuthBin(double azimuth) {
  int bin = static_cast<int>(std::floor(normalizeAzimuth(azimuth) /
                                        kSupportBinWidthRad));
  if (bin < 0) {
    bin = 0;
  }
  return std::min(kSupportBinCount - 1, bin);
}

void buildAzimuthIndex(PairStrip& strip) {
  for (std::size_t index = 0; index < strip.quads.size(); ++index) {
    strip.azimuth_bins[azimuthBin(strip.quads[index].midpoint_azimuth)]
        .push_back(index);
  }
}

bool neighborSupportsLongQuad(const QuadCandidate& candidate,
                              const PairStrip& neighbor,
                              const ScanStripSupportConfig& config) {
  const int center_bin = azimuthBin(candidate.midpoint_azimuth);
  const float normal_cosine = static_cast<float>(
      std::cos(radians(config.maximum_neighbor_normal_angle_deg)));
  for (int offset = -2; offset <= 2; ++offset) {
    int bin = (center_bin + offset) % kSupportBinCount;
    if (bin < 0) {
      bin += kSupportBinCount;
    }
    const auto found = neighbor.azimuth_bins.find(bin);
    if (found == neighbor.azimuth_bins.end()) {
      continue;
    }
    for (const std::size_t neighbor_index : found->second) {
      const QuadCandidate& support = neighbor.quads[neighbor_index];
      if (!support.locally_valid ||
          candidate.normal.dot(support.normal) < normal_cosine ||
          candidate.cross_direction.dot(support.cross_direction) <
              normal_cosine) {
        continue;
      }
      const double center_limit =
          0.5 * (candidate.azimuth_width + support.azimuth_width) +
          radians(config.maximum_match_angle_deg);
      if (circularDistance(candidate.midpoint_azimuth,
                           support.midpoint_azimuth) > center_limit) {
        continue;
      }
      const float plane_limit = std::max(
          config.voxel_size,
          config.maximum_neighbor_plane_edge_ratio *
              std::max(candidate.maximum_cross_edge,
                       support.maximum_cross_edge));
      const Eigen::Vector3f center =
          0.25f * (candidate.lower_start + candidate.lower_end +
                   candidate.upper_start + candidate.upper_end);
      const float maximum_distance = std::max(
          std::max(std::fabs((support.lower_start - center).dot(candidate.normal)),
                   std::fabs((support.lower_end - center).dot(candidate.normal))),
          std::max(std::fabs((support.upper_start - center).dot(candidate.normal)),
                   std::fabs((support.upper_end - center).dot(candidate.normal))));
      if (maximum_distance <= plane_limit) {
        return true;
      }
    }
  }
  return false;
}

SurfaceCellKey cellForPoint(const Eigen::Vector3f& point, float voxel_size) {
  const double inverse = 1.0 / static_cast<double>(voxel_size);
  const auto quantize = [inverse](float value) {
    const double scaled = std::floor(static_cast<double>(value) * inverse);
    return static_cast<std::int32_t>(std::max(
        static_cast<double>(std::numeric_limits<std::int32_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<std::int32_t>::max()),
                 scaled)));
  };
  return SurfaceCellKey{quantize(point.x()), quantize(point.y()),
                        quantize(point.z())};
}

Eigen::Vector3f centerForCell(const SurfaceCellKey& key, float voxel_size) {
  const float half = 0.5f * voxel_size;
  return Eigen::Vector3f(static_cast<float>(key.x) * voxel_size + half,
                         static_cast<float>(key.y) * voxel_size + half,
                         static_cast<float>(key.z) * voxel_size + half);
}

bool candidateBetter(const SurfaceCandidate& candidate,
                     const SurfaceCandidate& existing) {
  if (candidate.verified_long_span != existing.verified_long_span) {
    return !candidate.verified_long_span;
  }
  if (candidate.center_distance_squared != existing.center_distance_squared) {
    return candidate.center_distance_squared < existing.center_distance_squared;
  }
  if (candidate.lower_ring != existing.lower_ring) {
    return candidate.lower_ring < existing.lower_ring;
  }
  if (candidate.azimuth != existing.azimuth) {
    return candidate.azimuth < existing.azimuth;
  }
  return std::tie(candidate.position.x(), candidate.position.y(),
                  candidate.position.z()) <
         std::tie(existing.position.x(), existing.position.y(),
                  existing.position.z());
}

void addSurfaceSample(std::size_t sensor_index,
                      const Eigen::Vector3f& position,
                      std::uint16_t lower_ring, float azimuth,
                      bool verified_long_span,
                      const ScanStripSupportConfig& config,
                      std::vector<SurfaceCellMap>& cells,
                      ScanStripSupportResult& result) {
  ++result.stats.candidate_samples;
  if (!finitePoint(position)) {
    return;
  }
  const SurfaceCellKey key = cellForPoint(position, config.voxel_size);
  SurfaceCandidate candidate;
  candidate.position = position;
  candidate.lower_ring = lower_ring;
  candidate.azimuth = azimuth;
  candidate.verified_long_span = verified_long_span;
  candidate.center_distance_squared =
      (position - centerForCell(key, config.voxel_size)).squaredNorm();
  auto found = cells[sensor_index].find(key);
  if (found == cells[sensor_index].end()) {
    cells[sensor_index].emplace(key, candidate);
  } else if (candidateBetter(candidate, found->second)) {
    found->second = candidate;
  }
}

bool checkedRasterSteps(float length, double spacing, std::size_t limit,
                        std::size_t& steps) {
  const double step_count =
      std::ceil(static_cast<double>(length) / spacing);
  const std::size_t safe_limit =
      std::min(limit, std::numeric_limits<std::size_t>::max() - 1U);
  if (!std::isfinite(step_count) || step_count < 0.0 ||
      step_count >= static_cast<double>(
                        std::numeric_limits<std::size_t>::max()) ||
      step_count > static_cast<double>(safe_limit)) {
    return false;
  }
  steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(step_count));
  return steps <= safe_limit;
}

bool rasterSampleCount(const QuadCandidate& quad, bool include_end,
                       const ScanStripSupportConfig& config,
                       std::size_t& count) {
  const double spacing = static_cast<double>(config.voxel_size) *
                         static_cast<double>(config.surface_spacing_voxels);
  std::size_t u_steps = 0U;
  if (!checkedRasterSteps(quad.maximum_along_edge, spacing,
                          config.max_candidate_samples, u_steps)) {
    return false;
  }
  const std::size_t u_count = u_steps + (include_end ? 1U : 0U);
  count = 0U;
  for (std::size_t u_index = 0; u_index < u_count; ++u_index) {
    const float u = std::min(
        1.0f, static_cast<float>(u_index) / static_cast<float>(u_steps));
    const Eigen::Vector3f lower =
        quad.lower_start + u * (quad.lower_end - quad.lower_start);
    const Eigen::Vector3f upper =
        quad.upper_start + u * (quad.upper_end - quad.upper_start);
    std::size_t v_steps = 0U;
    if (!checkedRasterSteps((upper - lower).norm(), spacing,
                            config.max_candidate_samples, v_steps) ||
        v_steps + 1U > config.max_candidate_samples - count) {
      return false;
    }
    count += v_steps + 1U;
  }
  return true;
}

bool rasterizeQuad(std::size_t sensor_index, const QuadCandidate& quad,
                   bool include_end, const ScanStripSupportConfig& config,
                   std::vector<SurfaceCellMap>& cells,
                   ScanStripSupportResult& result) {
  const double spacing = static_cast<double>(config.voxel_size) *
                         static_cast<double>(config.surface_spacing_voxels);
  std::size_t u_steps = 0U;
  if (!checkedRasterSteps(quad.maximum_along_edge, spacing,
                          config.max_candidate_samples, u_steps)) {
    return false;
  }
  const std::size_t u_count = u_steps + (include_end ? 1U : 0U);
  for (std::size_t u_index = 0; u_index < u_count; ++u_index) {
    const float u = std::min(
        1.0f, static_cast<float>(u_index) / static_cast<float>(u_steps));
    const Eigen::Vector3f lower =
        quad.lower_start + u * (quad.lower_end - quad.lower_start);
    const Eigen::Vector3f upper =
        quad.upper_start + u * (quad.upper_end - quad.upper_start);
    const float cross_length = (upper - lower).norm();
    std::size_t v_steps = 0U;
    if (!checkedRasterSteps(cross_length, spacing,
                            config.max_candidate_samples, v_steps)) {
      return false;
    }
    const float azimuth = static_cast<float>(normalizeAzimuth(
        quad.start_azimuth + u * quad.azimuth_width));
    for (std::size_t v_index = 0; v_index <= v_steps; ++v_index) {
      const float v =
          static_cast<float>(v_index) / static_cast<float>(v_steps);
      const Eigen::Vector3f position = lower + v * (upper - lower);
      addSurfaceSample(sensor_index, position, quad.lower_ring, azimuth,
                       !quad.strong, config, cells, result);
    }
  }
  return true;
}

struct MeshVertexKey {
  std::uint8_t sensor_id = 0U;
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator<(const MeshVertexKey& other) const {
    return std::tie(sensor_id, x, y, z) <
           std::tie(other.sensor_id, other.x, other.y, other.z);
  }

  bool operator==(const MeshVertexKey& other) const {
    return sensor_id == other.sensor_id && x == other.x && y == other.y &&
           z == other.z;
  }
};

std::size_t hashCombine(std::size_t seed, std::size_t value) {
  return seed ^ (value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U));
}

struct MeshVertexKeyHash {
  std::size_t operator()(const MeshVertexKey& key) const {
    std::size_t hash = std::hash<unsigned int>{}(key.sensor_id);
    hash = hashCombine(hash, std::hash<std::int64_t>{}(key.x));
    hash = hashCombine(hash, std::hash<std::int64_t>{}(key.y));
    return hashCombine(hash, std::hash<std::int64_t>{}(key.z));
  }
};

struct MeshTriangleKeyHash {
  std::size_t operator()(const std::array<int, 3>& key) const {
    std::size_t hash = std::hash<int>{}(key[0]);
    hash = hashCombine(hash, std::hash<int>{}(key[1]));
    return hashCombine(hash, std::hash<int>{}(key[2]));
  }
};

struct MeshQuadPlan {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint8_t sensor_id = 0U;
  QuadCandidate quad;
  std::size_t start_breakpoint = 0U;
  std::size_t end_breakpoint = 0U;
  std::array<MeshVertexKey, 4> corners{};
};

struct MeshColumnKey {
  std::uint8_t sensor_id = 0U;
  std::size_t start_breakpoint = 0U;
  std::size_t end_breakpoint = 0U;

  bool operator<(const MeshColumnKey& other) const {
    return std::tie(sensor_id, start_breakpoint, end_breakpoint) <
           std::tie(other.sensor_id, other.start_breakpoint,
                    other.end_breakpoint);
  }
};

MeshColumnKey meshColumnKey(const MeshQuadPlan& plan) {
  return MeshColumnKey{plan.sensor_id, plan.start_breakpoint,
                       plan.end_breakpoint};
}

using MeshColumnSet = std::set<MeshColumnKey>;

using MeshQuadPlans =
    std::vector<MeshQuadPlan, Eigen::aligned_allocator<MeshQuadPlan>>;

struct MeshSupportComponent {
  std::uint16_t lower_ring = 0U;
  std::size_t component_id = 0U;

  bool operator==(const MeshSupportComponent& other) const {
    return lower_ring == other.lower_ring &&
           component_id == other.component_id;
  }
};

using QuadSupportComponent = std::vector<const QuadCandidate*>;
using QuadSupportComponents = std::vector<QuadSupportComponent>;

struct AtomicMeshColumn {
  std::size_t start_breakpoint = 0U;
  std::size_t end_breakpoint = 0U;
  double start_azimuth = 0.0;
  double end_azimuth = 0.0;
  double width = 0.0;
  bool usable = false;
  bool hard_cut = false;
  bool had_support = false;
  std::vector<MeshSupportComponent> signature;
  MeshQuadPlans plans;
};

struct SensorAtomicMeshData {
  std::size_t sensor_index = 0U;
  std::uint8_t sensor_id = 0U;
  std::vector<AtomicMeshColumn> columns;
};

using MergedAtomicColumns = std::map<MeshColumnKey, MeshColumnSet>;

struct CachedRingCurveSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool computed = false;
  bool valid = false;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
};

struct MeshBreakpointInterval {
  double start = 0.0;
  double end = 0.0;
  double width = 0.0;
  double midpoint = 0.0;
  bool usable = false;
};

using CachedRingCurveSamples =
    std::vector<CachedRingCurveSample,
                Eigen::aligned_allocator<CachedRingCurveSample>>;

struct CrossEdgeKey {
  std::uint8_t sensor_id = 0U;
  std::uint16_t lower_ring = 0U;
  std::size_t breakpoint = 0U;

  bool operator<(const CrossEdgeKey& other) const {
    return std::tie(sensor_id, lower_ring, breakpoint) <
           std::tie(other.sensor_id, other.lower_ring, other.breakpoint);
  }
};

struct CrossEdgePlan {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f lower = Eigen::Vector3f::Zero();
  Eigen::Vector3f upper = Eigen::Vector3f::Zero();
  std::size_t subdivisions = 1U;
  std::vector<std::size_t> vertices;
};

using CrossEdgePlans =
    std::map<CrossEdgeKey, CrossEdgePlan, std::less<CrossEdgeKey>,
             Eigen::aligned_allocator<
                 std::pair<const CrossEdgeKey, CrossEdgePlan>>>;

struct RingVertexKey {
  std::uint8_t sensor_id = 0U;
  std::uint16_t ring = 0U;
  std::size_t breakpoint = 0U;

  bool operator<(const RingVertexKey& other) const {
    return std::tie(sensor_id, ring, breakpoint) <
           std::tie(other.sensor_id, other.ring, other.breakpoint);
  }
};

struct IndexedMeshEdge {
  std::size_t first = 0U;
  std::size_t second = 0U;

  bool operator==(const IndexedMeshEdge& other) const {
    return first == other.first && second == other.second;
  }
};

IndexedMeshEdge indexedMeshEdge(std::size_t first, std::size_t second) {
  return first < second ? IndexedMeshEdge{first, second}
                        : IndexedMeshEdge{second, first};
}

struct IndexedMeshEdgeHash {
  std::size_t operator()(const IndexedMeshEdge& edge) const {
    return hashCombine(std::hash<std::size_t>{}(edge.first),
                       std::hash<std::size_t>{}(edge.second));
  }
};

bool meshVertexKey(std::uint8_t sensor_id, const Eigen::Vector3f& position,
                   double tolerance, MeshVertexKey& key) {
  if (!finitePoint(position) || !std::isfinite(tolerance) ||
      !(tolerance > 0.0)) {
    return false;
  }
  const auto quantize = [tolerance](float value, std::int64_t& output) {
    const double scaled = static_cast<double>(value) / tolerance;
    const double limit = static_cast<double>(
        std::numeric_limits<std::int64_t>::max() - 1);
    if (!std::isfinite(scaled) || std::fabs(scaled) >= limit) {
      return false;
    }
    output = static_cast<std::int64_t>(std::llround(scaled));
    return true;
  };
  key.sensor_id = sensor_id;
  return quantize(position.x(), key.x) && quantize(position.y(), key.y) &&
         quantize(position.z(), key.z);
}

bool vectorLexicographicallyLess(const Eigen::Vector3f& left,
                                 const Eigen::Vector3f& right) {
  return std::tie(left.x(), left.y(), left.z()) <
         std::tie(right.x(), right.y(), right.z());
}

void canonicalRotate(Eigen::Vector3i& triangle) {
  if (triangle.y() < triangle.x() && triangle.y() <= triangle.z()) {
    triangle = Eigen::Vector3i(triangle.y(), triangle.z(), triangle.x());
  } else if (triangle.z() < triangle.x() && triangle.z() < triangle.y()) {
    triangle = Eigen::Vector3i(triangle.z(), triangle.x(), triangle.y());
  }
}

struct VertexLinkEdge {
  std::size_t first = 0U;
  std::size_t second = 0U;

  bool operator<(const VertexLinkEdge& other) const {
    return std::tie(first, second) < std::tie(other.first, other.second);
  }

  bool operator==(const VertexLinkEdge& other) const {
    return first == other.first && second == other.second;
  }
};

bool vertexLinksAreSinglePathsOrRings(
    const std::vector<Eigen::Vector3i,
                      Eigen::aligned_allocator<Eigen::Vector3i>>& triangles,
    std::size_t vertex_count,
    std::vector<std::size_t>* invalid_vertices = nullptr) {
  if (invalid_vertices != nullptr) {
    invalid_vertices->clear();
  }
  std::vector<std::size_t> counts(vertex_count, 0U);
  for (const Eigen::Vector3i& triangle : triangles) {
    const std::array<int, 3> vertices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (const int vertex : vertices) {
      if (vertex < 0 || static_cast<std::size_t>(vertex) >= vertex_count) {
        return false;
      }
      ++counts[static_cast<std::size_t>(vertex)];
    }
  }

  std::vector<std::size_t> offsets(vertex_count + 1U, 0U);
  for (std::size_t vertex = 0; vertex < vertex_count; ++vertex) {
    if (counts[vertex] > std::numeric_limits<std::size_t>::max() -
                             offsets[vertex]) {
      return false;
    }
    offsets[vertex + 1U] = offsets[vertex] + counts[vertex];
  }
  std::vector<VertexLinkEdge> links(offsets.back());
  std::vector<std::size_t> cursors(offsets.begin(), offsets.end() - 1U);
  for (const Eigen::Vector3i& triangle : triangles) {
    const std::array<std::size_t, 3> vertices =
        {{static_cast<std::size_t>(triangle.x()),
          static_cast<std::size_t>(triangle.y()),
          static_cast<std::size_t>(triangle.z())}};
    for (std::size_t corner = 0; corner < 3U; ++corner) {
      std::size_t first = vertices[(corner + 1U) % 3U];
      std::size_t second = vertices[(corner + 2U) % 3U];
      if (second < first) {
        std::swap(first, second);
      }
      links[cursors[vertices[corner]]++] = VertexLinkEdge{first, second};
    }
  }

  std::vector<std::size_t> touched;
  std::vector<std::size_t> pending;
  std::vector<std::size_t> stamp(vertex_count,
                                 std::numeric_limits<std::size_t>::max());
  std::vector<std::size_t> visited_stamp(
      vertex_count, std::numeric_limits<std::size_t>::max());
  std::vector<std::uint8_t> degree(vertex_count, 0U);
  std::vector<std::array<std::size_t, 2>> adjacency(vertex_count);
  bool all_valid = true;
  for (std::size_t center = 0; center < vertex_count; ++center) {
    const auto begin =
        links.begin() + static_cast<std::ptrdiff_t>(offsets[center]);
    const auto end = links.begin() +
                     static_cast<std::ptrdiff_t>(offsets[center + 1U]);
    if (begin == end) {
      continue;
    }
    std::sort(begin, end);
    if (std::adjacent_find(begin, end) != end) {
      all_valid = false;
      if (invalid_vertices == nullptr) {
        return false;
      }
      invalid_vertices->push_back(center);
      continue;
    }
    touched.clear();
    const auto add_neighbor = [&](std::size_t from, std::size_t to) {
      if (stamp[from] != center) {
        stamp[from] = center;
        degree[from] = 0U;
        touched.push_back(from);
      }
      if (degree[from] >= 2U) {
        return false;
      }
      adjacency[from][degree[from]++] = to;
      return true;
    };
    bool center_valid = true;
    for (auto link = begin; link != end; ++link) {
      if (!add_neighbor(link->first, link->second) ||
          !add_neighbor(link->second, link->first)) {
        center_valid = false;
        break;
      }
    }
    if (!center_valid) {
      all_valid = false;
      if (invalid_vertices == nullptr) {
        return false;
      }
      invalid_vertices->push_back(center);
      continue;
    }
    std::size_t degree_one = 0U;
    for (const std::size_t vertex : touched) {
      if (degree[vertex] == 0U || degree[vertex] > 2U) {
        center_valid = false;
        break;
      }
      degree_one += degree[vertex] == 1U ? 1U : 0U;
    }
    if (!center_valid || (degree_one != 0U && degree_one != 2U)) {
      all_valid = false;
      if (invalid_vertices == nullptr) {
        return false;
      }
      invalid_vertices->push_back(center);
      continue;
    }
    pending.clear();
    pending.push_back(touched.front());
    visited_stamp[touched.front()] = center;
    std::size_t visited = 0U;
    while (!pending.empty()) {
      const std::size_t current = pending.back();
      pending.pop_back();
      ++visited;
      for (std::uint8_t index = 0U; index < degree[current]; ++index) {
        const std::size_t neighbor = adjacency[current][index];
        if (visited_stamp[neighbor] != center) {
          visited_stamp[neighbor] = center;
          pending.push_back(neighbor);
        }
      }
    }
    if (visited != touched.size()) {
      all_valid = false;
      if (invalid_vertices == nullptr) {
        return false;
      }
      invalid_vertices->push_back(center);
    }
  }
  return all_valid;
}

struct MeshRingSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeshRingSample() = default;
  MeshRingSample(double sample_azimuth,
                 const Eigen::Vector3f& sample_position)
      : azimuth(sample_azimuth), position(sample_position) {}

  double azimuth = 0.0;
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
};

using MeshRingSamples =
    std::vector<MeshRingSample, Eigen::aligned_allocator<MeshRingSample>>;

void canonicalizeRingSamples(MeshRingSamples& samples) {
  std::sort(samples.begin(), samples.end(),
            [](const MeshRingSample& left, const MeshRingSample& right) {
              if (left.azimuth != right.azimuth) {
                return left.azimuth < right.azimuth;
              }
              return vectorLexicographicallyLess(left.position,
                                                 right.position);
            });
  MeshRingSamples unique;
  unique.reserve(samples.size());
  for (const MeshRingSample& sample : samples) {
    if (!unique.empty() &&
        std::fabs(sample.azimuth - unique.back().azimuth) <=
            kDuplicateAzimuthEpsilon) {
      if (vectorLexicographicallyLess(sample.position,
                                      unique.back().position)) {
        unique.back().position = sample.position;
      }
      continue;
    }
    unique.push_back(sample);
  }
  samples.swap(unique);
}

bool sampleRingCurve(const MeshRingSamples& samples, double azimuth,
                     double maximum_gap, Eigen::Vector3f& position) {
  if (samples.size() < 2U) {
    return false;
  }
  azimuth = normalizeAzimuth(azimuth);
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), azimuth,
      [](const MeshRingSample& sample, double value) {
        return sample.azimuth < value;
      });
  if (upper != samples.end() &&
      std::fabs(upper->azimuth - azimuth) <= kDuplicateAzimuthEpsilon) {
    position = upper->position;
    return true;
  }
  const std::size_t upper_index =
      upper == samples.end()
          ? 0U
          : static_cast<std::size_t>(upper - samples.begin());
  const std::size_t lower_index =
      upper_index == 0U ? samples.size() - 1U : upper_index - 1U;
  const double gap = forwardAngle(samples[lower_index].azimuth,
                                  samples[upper_index].azimuth);
  const double offset =
      forwardAngle(samples[lower_index].azimuth, azimuth);
  if (!(gap > kDuplicateAzimuthEpsilon) || gap > maximum_gap ||
      offset > gap + kDuplicateAzimuthEpsilon) {
    return false;
  }
  const float fraction = static_cast<float>(offset / gap);
  position = samples[lower_index].position +
             fraction * (samples[upper_index].position -
                         samples[lower_index].position);
  return finitePoint(position);
}

bool nearbyBreakpointsTopologyCompatible(
    double first, double second,
    const std::map<std::uint16_t, MeshRingSamples>& ring_curves,
    double maximum_curve_gap, double weld_tolerance) {
  // Exact source breakpoints occur repeatedly across adjacent ring pairs.
  // Ring samples at the same canonical azimuth are already deterministic, so
  // avoid an O(rings * log(samples)) compatibility probe for that common case.
  if (forwardAngle(first, second) <= kDuplicateAzimuthEpsilon) {
    return true;
  }
  bool observed = false;
  for (const auto& curve : ring_curves) {
    Eigen::Vector3f first_position = Eigen::Vector3f::Zero();
    Eigen::Vector3f second_position = Eigen::Vector3f::Zero();
    const bool first_valid = sampleRingCurve(
        curve.second, first, maximum_curve_gap, first_position);
    const bool second_valid = sampleRingCurve(
        curve.second, second, maximum_curve_gap, second_position);
    if (first_valid != second_valid) {
      return false;
    }
    if (!first_valid) {
      continue;
    }
    observed = true;
    if (!finitePoint(first_position) || !finitePoint(second_position) ||
        static_cast<double>((first_position - second_position).norm()) >
            weld_tolerance) {
      return false;
    }
  }
  return observed;
}

using IndexedAzimuth = std::pair<double, std::size_t>;

constexpr std::int64_t kNoSupportComponent = -1;
constexpr std::int64_t kAmbiguousSupportComponent = -2;

void assignSupportComponentRange(
    const std::vector<IndexedAzimuth>& sorted_midpoints, double first,
    double last, std::int64_t component,
    std::vector<std::int64_t>& component_by_interval) {
  if (sorted_midpoints.empty() || first > last) {
    return;
  }
  const auto lower = std::lower_bound(
      sorted_midpoints.begin(), sorted_midpoints.end(), first,
      [](const IndexedAzimuth& value, double azimuth) {
        return value.first < azimuth;
      });
  const auto upper = std::upper_bound(
      sorted_midpoints.begin(), sorted_midpoints.end(), last,
      [](double azimuth, const IndexedAzimuth& value) {
        return azimuth < value.first;
      });
  for (auto item = lower; item != upper; ++item) {
    std::int64_t& existing = component_by_interval[item->second];
    if (existing == kNoSupportComponent || existing == component) {
      existing = component;
    } else {
      existing = kAmbiguousSupportComponent;
    }
  }
}

std::vector<std::int64_t> coveredBreakpointComponents(
    const QuadSupportComponents& components,
    const std::vector<IndexedAzimuth>& sorted_midpoints,
    std::size_t interval_count) {
  std::vector<std::int64_t> component_by_interval(
      interval_count, kNoSupportComponent);
  for (std::size_t component_index = 0U;
       component_index < components.size(); ++component_index) {
    const std::int64_t component =
        static_cast<std::int64_t>(component_index);
    for (const QuadCandidate* quad : components[component_index]) {
      if (quad == nullptr || !std::isfinite(quad->start_azimuth) ||
          !std::isfinite(quad->azimuth_width) ||
          !(quad->azimuth_width > 0.0)) {
        continue;
      }
      const double covered_width =
          quad->azimuth_width + kDuplicateAzimuthEpsilon;
      if (covered_width >= kTwoPi) {
        assignSupportComponentRange(sorted_midpoints, 0.0, kTwoPi,
                                    component, component_by_interval);
        continue;
      }
      const double start = normalizeAzimuth(quad->start_azimuth);
      const double end = start + covered_width;
      assignSupportComponentRange(sorted_midpoints, start,
                                  std::min(end, kTwoPi), component,
                                  component_by_interval);
      if (end >= kTwoPi) {
        assignSupportComponentRange(sorted_midpoints, 0.0, end - kTwoPi,
                                    component, component_by_interval);
      }
    }
  }
  return component_by_interval;
}

bool populateMeshPlanGeometry(const SensorCloud& sensor,
                              const ScanStripSupportConfig& config,
                              MeshQuadPlan& plan) {
  const std::array<float, 4> ranges = {
      {(plan.quad.lower_start - sensor.origin).norm(),
       (plan.quad.lower_end - sensor.origin).norm(),
       (plan.quad.upper_start - sensor.origin).norm(),
       (plan.quad.upper_end - sensor.origin).norm()}};
  return evaluateQuadGeometry(plan.quad, ranges, config);
}

enum class MeshPlanCornerStatus { kValid, kDegenerate, kOverflow };

MeshPlanCornerStatus populateMeshPlanCorners(MeshQuadPlan& plan,
                                             double weld_tolerance) {
  const std::array<Eigen::Vector3f, 4> positions = {
      {plan.quad.lower_start, plan.quad.lower_end, plan.quad.upper_start,
       plan.quad.upper_end}};
  for (std::size_t corner = 0U; corner < positions.size(); ++corner) {
    if (!meshVertexKey(plan.sensor_id, positions[corner], weld_tolerance,
                       plan.corners[corner])) {
      return MeshPlanCornerStatus::kOverflow;
    }
  }
  for (std::size_t first = 0U; first < plan.corners.size(); ++first) {
    for (std::size_t second = first + 1U; second < plan.corners.size();
         ++second) {
      if (plan.corners[first] == plan.corners[second]) {
        return MeshPlanCornerStatus::kDegenerate;
      }
    }
  }
  return MeshPlanCornerStatus::kValid;
}

MeshColumnKey atomicMeshColumnKey(const SensorAtomicMeshData& sensor,
                                  const AtomicMeshColumn& column) {
  return MeshColumnKey{sensor.sensor_id, column.start_breakpoint,
                       column.end_breakpoint};
}

bool makeMergedMeshPlans(const SensorCloud& sensor,
                         const std::vector<AtomicMeshColumn>& columns,
                         std::size_t begin, std::size_t end,
                         const ScanStripSupportConfig& config,
                         double target_along_edge, double weld_tolerance,
                         MeshQuadPlans& merged) {
  merged.clear();
  if (begin >= end || end > columns.size()) {
    return false;
  }
  const AtomicMeshColumn& first_column = columns[begin];
  const AtomicMeshColumn& last_column = columns[end - 1U];
  if (first_column.plans.empty() ||
      first_column.plans.size() != last_column.plans.size()) {
    return false;
  }
  const float normal_cosine = static_cast<float>(
      std::cos(radians(config.maximum_neighbor_normal_angle_deg)));
  const double edge_tolerance = 1e-5 * target_along_edge;
  merged.reserve(first_column.plans.size());
  for (std::size_t pair_index = 0U;
       pair_index < first_column.plans.size(); ++pair_index) {
    const MeshQuadPlan& first = first_column.plans[pair_index];
    const MeshQuadPlan& last = last_column.plans[pair_index];
    if (first.quad.lower_ring != last.quad.lower_ring) {
      return false;
    }
    MeshQuadPlan plan;
    plan.sensor_id = first.sensor_id;
    plan.start_breakpoint = first_column.start_breakpoint;
    plan.end_breakpoint = last_column.end_breakpoint;
    plan.quad.lower_ring = first.quad.lower_ring;
    plan.quad.lower_start = first.quad.lower_start;
    plan.quad.upper_start = first.quad.upper_start;
    plan.quad.lower_end = last.quad.lower_end;
    plan.quad.upper_end = last.quad.upper_end;
    plan.quad.start_azimuth = first_column.start_azimuth;
    plan.quad.end_azimuth = last_column.end_azimuth;
    plan.quad.azimuth_width =
        forwardAngle(plan.quad.start_azimuth, plan.quad.end_azimuth);
    plan.quad.midpoint_azimuth = normalizeAzimuth(
        plan.quad.start_azimuth + 0.5 * plan.quad.azimuth_width);
    plan.quad.lower_start_azimuth = plan.quad.start_azimuth;
    plan.quad.lower_end_azimuth = plan.quad.end_azimuth;
    plan.quad.upper_start_azimuth = plan.quad.start_azimuth;
    plan.quad.upper_end_azimuth = plan.quad.end_azimuth;
    if (!populateMeshPlanGeometry(sensor, config, plan) ||
        static_cast<double>(plan.quad.maximum_along_edge) >
            target_along_edge + edge_tolerance ||
        populateMeshPlanCorners(plan, weld_tolerance) !=
            MeshPlanCornerStatus::kValid) {
      return false;
    }
    for (std::size_t column_index = begin; column_index < end;
         ++column_index) {
      if (columns[column_index].plans.size() !=
              first_column.plans.size() ||
          columns[column_index].plans[pair_index].quad.lower_ring !=
              plan.quad.lower_ring) {
        return false;
      }
      const QuadCandidate& constituent =
          columns[column_index].plans[pair_index].quad;
      if (plan.quad.normal.dot(constituent.normal) < normal_cosine ||
          plan.quad.cross_direction.dot(constituent.cross_direction) <
              normal_cosine) {
        return false;
      }
    }
    merged.push_back(std::move(plan));
  }
  return true;
}

void appendCoarsenedSensorPlans(
    const SensorAtomicMeshData& sensor_data, const SensorCloud& sensor,
    const MeshColumnSet& masked_atomic_columns,
    const ScanStripSupportConfig& config, double target_along_edge,
    double weld_tolerance, MeshQuadPlans& plans,
    MergedAtomicColumns& merged_atomic_columns) {
  const std::vector<AtomicMeshColumn>& columns = sensor_data.columns;
  const auto available = [&sensor_data, &masked_atomic_columns](
                             const AtomicMeshColumn& column) {
    return column.usable && !column.hard_cut && !column.signature.empty() &&
           !column.plans.empty() &&
           masked_atomic_columns.count(
               atomicMeshColumnKey(sensor_data, column)) == 0U;
  };
  const auto append_group =
      [&sensor_data, &columns, &plans, &merged_atomic_columns](
          std::size_t begin, std::size_t end, MeshQuadPlans group_plans) {
        const MeshColumnKey merged_key{
            sensor_data.sensor_id, columns[begin].start_breakpoint,
            columns[end - 1U].end_breakpoint};
        MeshColumnSet& atomics = merged_atomic_columns[merged_key];
        for (std::size_t index = begin; index < end; ++index) {
          atomics.insert(atomicMeshColumnKey(sensor_data, columns[index]));
        }
        plans.insert(plans.end(),
                     std::make_move_iterator(group_plans.begin()),
                     std::make_move_iterator(group_plans.end()));
      };
  std::size_t run_begin = 0U;
  while (run_begin < columns.size()) {
    if (!available(columns[run_begin])) {
      ++run_begin;
      continue;
    }
    std::size_t run_end = run_begin + 1U;
    while (run_end < columns.size() && available(columns[run_end]) &&
           columns[run_end].signature == columns[run_begin].signature) {
      ++run_end;
    }

    // A topology failure at an asynchronous support-domain transition must
    // only retire the touching atomic column, not a long coarsened interior.
    // Keep one rollback guard column at each real run boundary.  A continuous
    // full-circle run has no real boundary; breakpoint zero remains only the
    // deterministic planning seam and therefore needs no guard.
    const bool wraps_same_signature =
        run_begin == 0U && run_end == columns.size() &&
        available(columns.front()) && available(columns.back()) &&
        columns.front().signature == columns.back().signature;
    std::size_t merge_begin = run_begin;
    std::size_t merge_end = run_end;
    if (!wraps_same_signature) {
      append_group(run_begin, run_begin + 1U, columns[run_begin].plans);
      merge_begin = run_begin + 1U;
      if (merge_begin < run_end) {
        append_group(run_end - 1U, run_end, columns[run_end - 1U].plans);
        merge_end = run_end - 1U;
      }
    }

    std::size_t group_begin = merge_begin;
    while (group_begin < merge_end) {
      const std::size_t pair_count = columns[group_begin].plans.size();
      std::vector<double> lower_path(pair_count, 0.0);
      std::vector<double> upper_path(pair_count, 0.0);
      std::vector<double> next_lower(pair_count, 0.0);
      std::vector<double> next_upper(pair_count, 0.0);
      std::size_t path_end = group_begin;
      while (path_end < merge_end) {
        const AtomicMeshColumn& column = columns[path_end];
        if (column.plans.size() != pair_count) {
          break;
        }
        bool path_valid = true;
        for (std::size_t pair_index = 0U; pair_index < pair_count;
             ++pair_index) {
          const QuadCandidate& quad = column.plans[pair_index].quad;
          next_lower[pair_index] = lower_path[pair_index] +
              static_cast<double>(
                  (quad.lower_end - quad.lower_start).norm());
          next_upper[pair_index] = upper_path[pair_index] +
              static_cast<double>(
                  (quad.upper_end - quad.upper_start).norm());
          if (!std::isfinite(next_lower[pair_index]) ||
              !std::isfinite(next_upper[pair_index]) ||
              next_lower[pair_index] > target_along_edge ||
              next_upper[pair_index] > target_along_edge) {
            path_valid = false;
            break;
          }
        }
        if (!path_valid) {
          break;
        }
        lower_path.swap(next_lower);
        upper_path.swap(next_upper);
        ++path_end;
      }

      if (path_end == group_begin) {
        append_group(group_begin, group_begin + 1U,
                     columns[group_begin].plans);
        ++group_begin;
        continue;
      }
      MeshQuadPlans merged;
      if (makeMergedMeshPlans(sensor, columns, group_begin, path_end, config,
                              target_along_edge, weld_tolerance, merged)) {
        append_group(group_begin, path_end, std::move(merged));
      } else {
        // Every atomic plan was independently gated.  If the direct chord of
        // their longest 0.5L prefix fails the merged geometry/normal test,
        // preserve that complete prefix atomically instead of repeatedly
        // testing shorter prefixes.  This is fail-closed and keeps work
        // linear even on adversarial folded strips.
        for (std::size_t index = group_begin; index < path_end; ++index) {
          append_group(index, index + 1U, columns[index].plans);
        }
      }
      group_begin = path_end;
    }
    run_begin = run_end;
  }
}

using RawMeshTriangle = std::array<std::size_t, 3>;
using RawMeshTriangles = std::vector<RawMeshTriangle>;

struct MeshColumnEvidence {
  std::size_t strong_plans = 0U;
  std::size_t plans = 0U;
  double closest_range = std::numeric_limits<double>::infinity();
};

using MeshColumnEvidenceMap = std::map<MeshColumnKey, MeshColumnEvidence>;

struct OutputTopologyFan {
  MeshColumnSet columns;
  std::size_t strong_plans = 0U;
  std::size_t plan_evidence = 0U;
  std::size_t incident_triangles = 0U;
  double closest_range = std::numeric_limits<double>::infinity();
};

// A valid manifold edge has at most two incident triangles. Keep that common
// case entirely inside the hash-table node; only an already-conflicting edge
// allocates overflow storage. The indexed access below deliberately preserves
// the original triangle insertion order used by the conflict resolver.
struct OutputEdgeIncidents {
  std::array<std::size_t, 2> inline_triangles{{0U, 0U}};
  std::vector<std::size_t> overflow_triangles;
  std::size_t count = 0U;

  void pushBack(std::size_t triangle) {
    if (count < inline_triangles.size()) {
      inline_triangles[count] = triangle;
    } else {
      overflow_triangles.push_back(triangle);
    }
    ++count;
  }

  std::size_t size() const { return count; }

  std::size_t operator[](std::size_t index) const {
    return index < inline_triangles.size()
               ? inline_triangles[index]
               : overflow_triangles[index - inline_triangles.size()];
  }
};

MeshColumnEvidenceMap meshColumnEvidence(
    const SensorClouds& sensors, const MeshQuadPlans& plans) {
  std::array<const SensorCloud*, 256> sensor_by_id{};
  for (const SensorCloud& sensor : sensors) {
    sensor_by_id[sensor.sensor_id] = &sensor;
  }
  MeshColumnEvidenceMap evidence;
  for (const MeshQuadPlan& plan : plans) {
    MeshColumnEvidence& column = evidence[meshColumnKey(plan)];
    column.strong_plans += plan.quad.strong ? 1U : 0U;
    ++column.plans;
    const SensorCloud* sensor = sensor_by_id[plan.sensor_id];
    if (sensor == nullptr) {
      continue;
    }
    const std::array<Eigen::Vector3f, 4> corners = {
        {plan.quad.lower_start, plan.quad.lower_end,
         plan.quad.upper_start, plan.quad.upper_end}};
    for (const Eigen::Vector3f& corner : corners) {
      const double range =
          static_cast<double>((corner - sensor->origin).norm());
      if (std::isfinite(range)) {
        column.closest_range = std::min(column.closest_range, range);
      }
    }
  }
  return evidence;
}

OutputTopologyFan outputTopologyFan(
    const std::vector<std::size_t>& triangle_indices,
    const std::vector<MeshColumnKey>& raw_triangle_columns,
    const MeshColumnEvidenceMap& evidence) {
  OutputTopologyFan fan;
  fan.incident_triangles = triangle_indices.size();
  for (const std::size_t triangle : triangle_indices) {
    if (triangle < raw_triangle_columns.size()) {
      fan.columns.insert(raw_triangle_columns[triangle]);
    }
  }
  for (const MeshColumnKey& column : fan.columns) {
    const auto found = evidence.find(column);
    if (found == evidence.end()) {
      continue;
    }
    fan.strong_plans += found->second.strong_plans;
    fan.plan_evidence += found->second.plans;
    fan.closest_range =
        std::min(fan.closest_range, found->second.closest_range);
  }
  return fan;
}

bool outputTopologyFanIsBetter(const OutputTopologyFan& candidate,
                               const OutputTopologyFan& incumbent) {
  if (candidate.strong_plans != incumbent.strong_plans) {
    return candidate.strong_plans > incumbent.strong_plans;
  }
  if (candidate.closest_range != incumbent.closest_range) {
    return candidate.closest_range < incumbent.closest_range;
  }
  if (candidate.columns.size() != incumbent.columns.size()) {
    return candidate.columns.size() > incumbent.columns.size();
  }
  if (candidate.incident_triangles != incumbent.incident_triangles) {
    return candidate.incident_triangles > incumbent.incident_triangles;
  }
  if (candidate.plan_evidence != incumbent.plan_evidence) {
    return candidate.plan_evidence > incumbent.plan_evidence;
  }
  return std::lexicographical_compare(
      candidate.columns.begin(), candidate.columns.end(),
      incumbent.columns.begin(), incumbent.columns.end());
}

void maskAllButOneOutputTopologyFan(
    const std::vector<OutputTopologyFan>& fans,
    MeshColumnSet& masked_columns) {
  if (fans.empty()) {
    return;
  }
  std::map<MeshColumnKey, std::size_t> owners;
  for (const OutputTopologyFan& fan : fans) {
    for (const MeshColumnKey& column : fan.columns) {
      ++owners[column];
    }
  }
  bool shared_column = false;
  for (const auto& owner : owners) {
    if (owner.second > 1U) {
      masked_columns.insert(owner.first);
      shared_column = true;
    }
  }
  if (shared_column) {
    // A complete sensor/azimuth column is the smallest legal deletion unit.
    // If it owns faces in several fans, retaining one fan while deleting the
    // other is impossible without a partial-column edit.  Remove only those
    // shared columns and let the next bounded rebuild recompute the fans.
    return;
  }
  if (fans.size() == 1U) {
    masked_columns.insert(fans.front().columns.begin(),
                          fans.front().columns.end());
    return;
  }
  std::size_t winner = 0U;
  for (std::size_t index = 1U; index < fans.size(); ++index) {
    if (outputTopologyFanIsBetter(fans[index], fans[winner])) {
      winner = index;
    }
  }
  for (std::size_t index = 0U; index < fans.size(); ++index) {
    if (index != winner) {
      masked_columns.insert(fans[index].columns.begin(),
                            fans[index].columns.end());
    }
  }
}

bool collectOutputTopologyConflictColumns(
    const SupportedStripMesh& mesh, const RawMeshTriangles& raw_triangles,
    const std::vector<MeshColumnKey>& raw_triangle_columns,
    const std::vector<std::uint8_t>& vertex_sensor_ids,
    const SensorClouds& sensors, const MeshQuadPlans& plans,
    MeshColumnSet& masked_columns, std::string* fatal_reason) {
  masked_columns.clear();
  if (fatal_reason != nullptr) {
    fatal_reason->clear();
  }
  if (raw_triangles.size() != raw_triangle_columns.size() ||
      mesh.vertices.size() != vertex_sensor_ids.size()) {
    if (fatal_reason != nullptr) {
      *fatal_reason = "mesh_output_equivalence_input_mismatch";
    }
    return false;
  }
  const MeshColumnEvidenceMap evidence = meshColumnEvidence(sensors, plans);

  std::vector<std::size_t> canonical_vertex(mesh.vertices.size(), 0U);
  std::unordered_map<MeshVertexKey, std::size_t, MeshVertexKeyHash>
      canonical_by_position;
  canonical_by_position.reserve(mesh.vertices.size());
  for (std::size_t vertex = 0U; vertex < mesh.vertices.size(); ++vertex) {
    MeshVertexKey key;
    if (!meshVertexKey(0U, mesh.vertices[vertex],
                       kOutputTopologyEquivalenceToleranceM, key)) {
      if (fatal_reason != nullptr) {
        *fatal_reason = "mesh_output_equivalence_coordinate_overflow";
      }
      return false;
    }
    const auto insertion = canonical_by_position.emplace(key, vertex);
    canonical_vertex[vertex] = insertion.first->second;
  }

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
      canonical_triangles;
  std::vector<std::size_t> canonical_to_raw;
  canonical_triangles.reserve(raw_triangles.size());
  canonical_to_raw.reserve(raw_triangles.size());
  for (std::size_t triangle = 0U; triangle < raw_triangles.size();
       ++triangle) {
    const RawMeshTriangle& raw = raw_triangles[triangle];
    if (raw[0] >= canonical_vertex.size() ||
        raw[1] >= canonical_vertex.size() ||
        raw[2] >= canonical_vertex.size()) {
      masked_columns.insert(raw_triangle_columns[triangle]);
      continue;
    }
    const std::array<std::size_t, 3> canonical = {
        {canonical_vertex[raw[0]], canonical_vertex[raw[1]],
         canonical_vertex[raw[2]]}};
    if (canonical[0] == canonical[1] || canonical[1] == canonical[2] ||
        canonical[0] == canonical[2] ||
        canonical[0] > static_cast<std::size_t>(
                           std::numeric_limits<int>::max()) ||
        canonical[1] > static_cast<std::size_t>(
                           std::numeric_limits<int>::max()) ||
        canonical[2] > static_cast<std::size_t>(
                           std::numeric_limits<int>::max())) {
      masked_columns.insert(raw_triangle_columns[triangle]);
      continue;
    }
    canonical_triangles.emplace_back(static_cast<int>(canonical[0]),
                                     static_cast<int>(canonical[1]),
                                     static_cast<int>(canonical[2]));
    canonical_to_raw.push_back(triangle);
  }

  const auto remove_masked_triangles =
      [&raw_triangle_columns, &masked_columns](
          std::vector<Eigen::Vector3i,
                      Eigen::aligned_allocator<Eigen::Vector3i>>& triangles,
          std::vector<std::size_t>& triangle_to_raw) {
        std::size_t output = 0U;
        for (std::size_t input = 0U; input < triangles.size(); ++input) {
          const std::size_t raw_index = triangle_to_raw[input];
          if (raw_index < raw_triangle_columns.size() &&
              masked_columns.count(raw_triangle_columns[raw_index]) == 0U) {
            if (output != input) {
              triangles[output] = triangles[input];
              triangle_to_raw[output] = raw_index;
            }
            ++output;
          }
        }
        triangles.resize(output);
        triangle_to_raw.resize(output);
      };
  remove_masked_triangles(canonical_triangles, canonical_to_raw);

  std::unordered_map<std::array<int, 3>, std::size_t, MeshTriangleKeyHash>
      face_winner;
  face_winner.reserve(canonical_triangles.size());
  for (std::size_t triangle = 0U; triangle < canonical_triangles.size();
       ++triangle) {
    std::array<int, 3> key = {
        {canonical_triangles[triangle].x(),
         canonical_triangles[triangle].y(),
         canonical_triangles[triangle].z()}};
    std::sort(key.begin(), key.end());
    const auto insertion = face_winner.emplace(key, triangle);
    if (insertion.second) {
      continue;
    }
    const std::vector<std::size_t> incumbent_indices = {
        canonical_to_raw[insertion.first->second]};
    const std::vector<std::size_t> candidate_indices = {
        canonical_to_raw[triangle]};
    const OutputTopologyFan incumbent = outputTopologyFan(
        incumbent_indices, raw_triangle_columns, evidence);
    const OutputTopologyFan candidate = outputTopologyFan(
        candidate_indices, raw_triangle_columns, evidence);
    if (outputTopologyFanIsBetter(candidate, incumbent)) {
      masked_columns.insert(incumbent.columns.begin(),
                            incumbent.columns.end());
      insertion.first->second = triangle;
    } else {
      masked_columns.insert(candidate.columns.begin(),
                            candidate.columns.end());
    }
  }
  remove_masked_triangles(canonical_triangles, canonical_to_raw);

  using CanonicalEdge = std::pair<std::size_t, std::size_t>;
  std::unordered_map<IndexedMeshEdge, OutputEdgeIncidents,
                     IndexedMeshEdgeHash>
      triangles_by_edge;
  if (canonical_triangles.size() <=
      std::numeric_limits<std::size_t>::max() / 3U) {
    triangles_by_edge.reserve(canonical_triangles.size() * 3U);
  }
  for (std::size_t triangle = 0U; triangle < canonical_triangles.size();
       ++triangle) {
    const Eigen::Vector3i& face = canonical_triangles[triangle];
    const std::array<std::size_t, 3> vertices = {
        {static_cast<std::size_t>(face.x()),
         static_cast<std::size_t>(face.y()),
         static_cast<std::size_t>(face.z())}};
    for (std::size_t edge = 0U; edge < 3U; ++edge) {
      const std::size_t first = vertices[edge];
      const std::size_t second = vertices[(edge + 1U) % 3U];
      triangles_by_edge[indexedMeshEdge(first, second)].pushBack(triangle);
    }
  }
  for (const auto& edge : triangles_by_edge) {
    if (edge.second.size() == 2U) {
      int directed_balance = 0;
      for (std::size_t incident = 0U; incident < edge.second.size();
           ++incident) {
        const std::size_t triangle = edge.second[incident];
        const Eigen::Vector3i& face = canonical_triangles[triangle];
        for (int corner = 0; corner < 3; ++corner) {
          const std::size_t first =
              static_cast<std::size_t>(face[corner]);
          const std::size_t second =
              static_cast<std::size_t>(face[(corner + 1) % 3]);
          if (std::min(first, second) == edge.first.first &&
              std::max(first, second) == edge.first.second) {
            directed_balance += first < second ? 1 : -1;
            break;
          }
        }
      }
      if (directed_balance != 0) {
        if (fatal_reason != nullptr) {
          *fatal_reason = "mesh_output_equivalence_winding_conflict";
        }
        return false;
      }
      continue;
    }
    if (edge.second.size() < 2U) {
      continue;
    }
    std::map<CanonicalEdge, std::vector<std::size_t>> triangles_by_raw_edge;
    for (std::size_t incident = 0U; incident < edge.second.size(); ++incident) {
      const std::size_t canonical_triangle = edge.second[incident];
      const std::size_t raw_index = canonical_to_raw[canonical_triangle];
      const RawMeshTriangle& raw = raw_triangles[raw_index];
      bool found_edge = false;
      for (std::size_t corner = 0U; corner < 3U; ++corner) {
        const std::size_t first = raw[corner];
        const std::size_t second = raw[(corner + 1U) % 3U];
        const CanonicalEdge canonical_edge(
            std::min(canonical_vertex[first], canonical_vertex[second]),
            std::max(canonical_vertex[first], canonical_vertex[second]));
        if (canonical_edge.first == edge.first.first &&
            canonical_edge.second == edge.first.second) {
          triangles_by_raw_edge[std::minmax(first, second)].push_back(
              raw_index);
          found_edge = true;
          break;
        }
      }
      if (!found_edge) {
        masked_columns.insert(raw_triangle_columns[raw_index]);
      }
    }
    std::vector<OutputTopologyFan> fans;
    fans.reserve(triangles_by_raw_edge.size());
    for (const auto& group : triangles_by_raw_edge) {
      fans.push_back(outputTopologyFan(group.second, raw_triangle_columns,
                                       evidence));
    }
    maskAllButOneOutputTopologyFan(fans, masked_columns);
  }
  remove_masked_triangles(canonical_triangles, canonical_to_raw);

  std::vector<std::size_t> invalid_vertices;
  if (vertexLinksAreSinglePathsOrRings(canonical_triangles,
                                       mesh.vertices.size(),
                                       &invalid_vertices)) {
    return !masked_columns.empty();
  }
  std::set<std::size_t> invalid_set(invalid_vertices.begin(),
                                    invalid_vertices.end());
  std::map<std::size_t, std::vector<std::size_t>> incident_by_center;
  for (std::size_t triangle = 0U; triangle < canonical_triangles.size();
       ++triangle) {
    const Eigen::Vector3i& face = canonical_triangles[triangle];
    for (int corner = 0; corner < 3; ++corner) {
      const std::size_t center =
          static_cast<std::size_t>(face[corner]);
      if (invalid_set.count(center) != 0U) {
        incident_by_center[center].push_back(triangle);
      }
    }
  }
  for (const auto& incident : incident_by_center) {
    std::map<std::size_t, std::vector<std::size_t>> triangles_by_raw_center;
    for (const std::size_t canonical_triangle : incident.second) {
      const std::size_t raw_index = canonical_to_raw[canonical_triangle];
      const RawMeshTriangle& raw = raw_triangles[raw_index];
      std::size_t raw_center = mesh.vertices.size();
      for (const std::size_t vertex : raw) {
        if (canonical_vertex[vertex] == incident.first) {
          raw_center = vertex;
          break;
        }
      }
      if (raw_center >= mesh.vertices.size()) {
        masked_columns.insert(raw_triangle_columns[raw_index]);
      } else {
        triangles_by_raw_center[raw_center].push_back(raw_index);
      }
    }
    std::vector<OutputTopologyFan> fans;
    fans.reserve(triangles_by_raw_center.size());
    for (const auto& group : triangles_by_raw_center) {
      fans.push_back(outputTopologyFan(group.second, raw_triangle_columns,
                                       evidence));
    }
    maskAllButOneOutputTopologyFan(fans, masked_columns);
  }
  return !masked_columns.empty();
}

MeshTopologyValidationResult validateOutputTopology(
    SupportedStripMesh& mesh) {
  IndexedMesh indexed;
  indexed.vertices.swap(mesh.vertices);
  indexed.triangles.swap(mesh.triangles);
  const MeshTopologyValidationResult result =
      validateMeshTopology(indexed, kOutputTopologyEquivalenceToleranceM);
  indexed.vertices.swap(mesh.vertices);
  indexed.triangles.swap(mesh.triangles);
  return result;
}

enum class MeshFinalizeStatus {
  kAccepted,
  kExactTopologyInvalid,
  kOutputTopologyInvalid,
};

template <typename FinalizeMesh>
bool repairOutputTopologyColumns(
    const SupportedStripMesh& source_mesh,
    const RawMeshTriangles& raw_triangles,
    const std::vector<MeshColumnKey>& raw_triangle_columns,
    const std::vector<std::uint8_t>& vertex_sensor_ids,
    const SensorClouds& sensors, const MeshQuadPlans& plans,
    FinalizeMesh& finalize_mesh, MeshColumnSet& output_filtered_columns,
    RawMeshTriangles& active_triangles,
    std::vector<MeshColumnKey>& active_columns,
    SupportedStripMesh& finalized_mesh, bool& output_mesh_finalized,
    std::string& failure_reason,
    std::vector<MeshColumnSet>* repair_trace = nullptr) {
  output_filtered_columns.clear();
  active_triangles.clear();
  active_columns.clear();
  finalized_mesh = SupportedStripMesh();
  output_mesh_finalized = false;
  failure_reason.clear();
  const MeshColumnSet unique_columns(raw_triangle_columns.begin(),
                                     raw_triangle_columns.end());
  for (std::size_t iteration = 0U; iteration <= unique_columns.size();
       ++iteration) {
    active_triangles.clear();
    active_columns.clear();
    active_triangles.reserve(raw_triangles.size());
    active_columns.reserve(raw_triangle_columns.size());
    for (std::size_t triangle = 0U; triangle < raw_triangles.size();
         ++triangle) {
      if (output_filtered_columns.count(raw_triangle_columns[triangle]) ==
          0U) {
        active_triangles.push_back(raw_triangles[triangle]);
        active_columns.push_back(raw_triangle_columns[triangle]);
      }
    }
    if (!output_filtered_columns.empty()) {
      SupportedStripMesh optimistic_mesh;
      MeshColumnSet ignored_exact_conflicts;
      std::string ignored_reason;
      const MeshFinalizeStatus optimistic_status = finalize_mesh(
          source_mesh, active_triangles, active_columns, optimistic_mesh,
          ignored_exact_conflicts, ignored_reason);
      if (optimistic_status == MeshFinalizeStatus::kAccepted) {
        // The independent 0.1 mm output validator proves that the filtered
        // mesh has no remaining equivalence conflict. Keep the collector below
        // as the fail-safe repair path whenever this proof fails.
        finalized_mesh = std::move(optimistic_mesh);
        output_mesh_finalized = true;
        return true;
      }
    }
    MeshColumnSet conflicts;
    std::string fatal_reason;
    const bool has_conflicts = collectOutputTopologyConflictColumns(
        source_mesh, active_triangles, active_columns, vertex_sensor_ids,
        sensors, plans, conflicts, &fatal_reason);
    if (!fatal_reason.empty()) {
      failure_reason = std::move(fatal_reason);
      return false;
    }
    if (!has_conflicts) {
      if (!conflicts.empty()) {
        failure_reason = "mesh_output_equivalence_state_mismatch";
        return false;
      }
      return true;
    }
    if (repair_trace != nullptr) {
      repair_trace->push_back(conflicts);
    }
    std::size_t added = 0U;
    for (const MeshColumnKey& conflict : conflicts) {
      added += output_filtered_columns.insert(conflict).second ? 1U : 0U;
    }
    if (added == 0U) {
      failure_reason = "mesh_output_column_filter_stalled";
      return false;
    }
  }
  failure_reason = "mesh_output_column_filter_iteration_limit";
  return false;
}

SupportedStripMesh buildZipperStripMesh(
    const SensorClouds& sensors, const MeshQuadPlans& plans,
    const ScanStripSupportConfig& config, double maximum_edge,
    double weld_tolerance, SupportedStripMesh mesh,
    MeshColumnSet* nonmanifold_columns) {
  if (nonmanifold_columns != nullptr) {
    nonmanifold_columns->clear();
  }
  const float maximum_edge_with_tolerance =
      static_cast<float>(maximum_edge + 1e-5 * maximum_edge);
  const auto fail = [&mesh](const char* reason, bool budget_limited = false) {
    mesh.accepted = false;
    mesh.budget_limited = budget_limited;
    mesh.reason = reason;
    mesh.vertices.clear();
    mesh.triangles.clear();
    return mesh;
  };
  const auto topology_positions_match =
      [weld_tolerance](const Eigen::Vector3f& first,
                       const Eigen::Vector3f& second) {
        return finitePoint(first) && finitePoint(second) &&
               static_cast<double>((first - second).norm()) <=
                   weld_tolerance;
      };

  CrossEdgePlans cross_edges;
  for (const MeshQuadPlan& plan : plans) {
    const double lower_along = static_cast<double>(
        (plan.quad.lower_end - plan.quad.lower_start).norm());
    const double upper_along = static_cast<double>(
        (plan.quad.upper_end - plan.quad.upper_start).norm());
    const double left_cross = static_cast<double>(
        (plan.quad.upper_start - plan.quad.lower_start).norm());
    const double right_cross = static_cast<double>(
        (plan.quad.upper_end - plan.quad.lower_end).norm());
    const double along_bound = std::max(lower_along, upper_along);
    const double cross_bound = std::max(left_cross, right_cross);
    if (!std::isfinite(along_bound) || !std::isfinite(cross_bound) ||
        !(cross_bound > 0.0)) {
      return fail("mesh_zipper_nonfinite_geometry");
    }
    const double subdivision_slack =
        static_cast<double>(maximum_edge_with_tolerance) - along_bound;
    if (!(subdivision_slack > 0.0) ||
        !std::isfinite(subdivision_slack)) {
      return fail("mesh_zipper_along_edge_limit");
    }
    const double required_value = cross_bound / subdivision_slack;
    if (!std::isfinite(required_value) ||
        required_value >
            static_cast<double>(config.max_mesh_vertices)) {
      return fail("mesh_budget_limited", true);
    }
    const std::size_t required_subdivisions = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(required_value)));

    const std::array<CrossEdgeKey, 2> keys =
        {{{plan.sensor_id, plan.quad.lower_ring, plan.start_breakpoint},
          {plan.sensor_id, plan.quad.lower_ring, plan.end_breakpoint}}};
    const std::array<Eigen::Vector3f, 2> lower =
        {{plan.quad.lower_start, plan.quad.lower_end}};
    const std::array<Eigen::Vector3f, 2> upper =
        {{plan.quad.upper_start, plan.quad.upper_end}};
    for (std::size_t side = 0U; side < keys.size(); ++side) {
      auto insertion = cross_edges.emplace(keys[side], CrossEdgePlan());
      CrossEdgePlan& edge = insertion.first->second;
      if (insertion.second) {
        edge.lower = lower[side];
        edge.upper = upper[side];
      } else if (!topology_positions_match(edge.lower, lower[side]) ||
                 !topology_positions_match(edge.upper, upper[side])) {
        return fail("mesh_topology_edge_mismatch");
      }
      edge.subdivisions =
          std::max(edge.subdivisions, required_subdivisions);
    }
  }

  std::set<RingVertexKey> ring_keys;
  std::size_t expected_vertices = 0U;
  for (const auto& entry : cross_edges) {
    const CrossEdgeKey& key = entry.first;
    const CrossEdgePlan& edge = entry.second;
    ring_keys.insert(
        RingVertexKey{key.sensor_id, key.lower_ring, key.breakpoint});
    if (key.lower_ring == std::numeric_limits<std::uint16_t>::max()) {
      return fail("mesh_ring_index_overflow");
    }
    ring_keys.insert(RingVertexKey{
        key.sensor_id, static_cast<std::uint16_t>(key.lower_ring + 1U),
        key.breakpoint});
    const std::size_t interior = edge.subdivisions - 1U;
    if (expected_vertices > config.max_mesh_vertices ||
        interior > config.max_mesh_vertices - expected_vertices) {
      return fail("mesh_budget_limited", true);
    }
    expected_vertices += interior;
  }
  if (expected_vertices > config.max_mesh_vertices ||
      ring_keys.size() > config.max_mesh_vertices - expected_vertices) {
    return fail("mesh_budget_limited", true);
  }
  expected_vertices += ring_keys.size();
  if (expected_vertices > config.max_mesh_vertices ||
      expected_vertices >
          static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return fail("mesh_budget_limited", true);
  }

  std::size_t expected_triangles = 0U;
  for (const MeshQuadPlan& plan : plans) {
    const CrossEdgeKey left_key{plan.sensor_id, plan.quad.lower_ring,
                                plan.start_breakpoint};
    const CrossEdgeKey right_key{plan.sensor_id, plan.quad.lower_ring,
                                 plan.end_breakpoint};
    const auto left = cross_edges.find(left_key);
    const auto right = cross_edges.find(right_key);
    if (left == cross_edges.end() || right == cross_edges.end()) {
      return fail("mesh_missing_cross_edge");
    }
    if (right->second.subdivisions >
        std::numeric_limits<std::size_t>::max() -
            left->second.subdivisions) {
      return fail("mesh_budget_limited", true);
    }
    const std::size_t added =
        left->second.subdivisions + right->second.subdivisions;
    if (expected_triangles > config.max_mesh_triangles ||
        added > config.max_mesh_triangles - expected_triangles) {
      return fail("mesh_budget_limited", true);
    }
    expected_triangles += added;
  }
  if (expected_triangles > config.max_mesh_triangles) {
    return fail("mesh_budget_limited", true);
  }

  mesh.vertices.reserve(expected_vertices);
  std::vector<std::uint8_t> vertex_sensor_ids;
  vertex_sensor_ids.reserve(expected_vertices);
  std::map<RingVertexKey, std::size_t> ring_vertices;
  const auto add_vertex = [&mesh, &vertex_sensor_ids](
                              std::uint8_t sensor_id,
                              const Eigen::Vector3f& position) {
    const std::size_t index = mesh.vertices.size();
    mesh.vertices.push_back(position);
    vertex_sensor_ids.push_back(sensor_id);
    return index;
  };
  for (auto& entry : cross_edges) {
    const CrossEdgeKey& key = entry.first;
    CrossEdgePlan& edge = entry.second;
    edge.vertices.resize(edge.subdivisions + 1U);
    for (std::size_t step = 0U; step <= edge.subdivisions; ++step) {
      const float fraction = static_cast<float>(step) /
                             static_cast<float>(edge.subdivisions);
      const Eigen::Vector3f position =
          edge.lower + fraction * (edge.upper - edge.lower);
      if (!finitePoint(position)) {
        return fail("mesh_nonfinite_zipper_vertex");
      }
      if (step != 0U && step != edge.subdivisions) {
        edge.vertices[step] = add_vertex(key.sensor_id, position);
        continue;
      }
      const std::uint16_t ring =
          step == 0U
              ? key.lower_ring
              : static_cast<std::uint16_t>(key.lower_ring + 1U);
      const RingVertexKey ring_key{key.sensor_id, ring, key.breakpoint};
      const auto existing = ring_vertices.find(ring_key);
      if (existing != ring_vertices.end()) {
        if (!topology_positions_match(mesh.vertices[existing->second],
                                      position)) {
          return fail("mesh_topology_vertex_mismatch");
        }
        edge.vertices[step] = existing->second;
      } else {
        const std::size_t index = add_vertex(key.sensor_id, position);
        ring_vertices.emplace(ring_key, index);
        edge.vertices[step] = index;
      }
    }
  }
  if (mesh.vertices.size() != expected_vertices) {
    return fail("mesh_zipper_vertex_count_mismatch");
  }

  RawMeshTriangles raw_triangles;
  raw_triangles.reserve(expected_triangles);
  std::vector<MeshColumnKey> raw_triangle_columns;
  raw_triangle_columns.reserve(expected_triangles);
  for (const MeshQuadPlan& plan : plans) {
    const CrossEdgeKey left_key{plan.sensor_id, plan.quad.lower_ring,
                                plan.start_breakpoint};
    const CrossEdgeKey right_key{plan.sensor_id, plan.quad.lower_ring,
                                 plan.end_breakpoint};
    const CrossEdgePlan& left = cross_edges.find(left_key)->second;
    const CrossEdgePlan& right = cross_edges.find(right_key)->second;
    std::size_t left_step = 0U;
    std::size_t right_step = 0U;
    while (left_step < left.subdivisions ||
           right_step < right.subdivisions) {
      bool advance_right = left_step == left.subdivisions;
      if (right_step == right.subdivisions) {
        advance_right = false;
      } else if (left_step < left.subdivisions) {
        const std::uint64_t next_left =
            static_cast<std::uint64_t>(left_step + 1U) *
            static_cast<std::uint64_t>(right.subdivisions);
        const std::uint64_t next_right =
            static_cast<std::uint64_t>(right_step + 1U) *
            static_cast<std::uint64_t>(left.subdivisions);
        advance_right = next_right <= next_left;
      }
      if (advance_right) {
        raw_triangles.push_back(
            {{left.vertices[left_step], right.vertices[right_step],
              right.vertices[right_step + 1U]}});
        raw_triangle_columns.push_back(meshColumnKey(plan));
        ++right_step;
      } else {
        raw_triangles.push_back(
            {{left.vertices[left_step], right.vertices[right_step],
              left.vertices[left_step + 1U]}});
        raw_triangle_columns.push_back(meshColumnKey(plan));
        ++left_step;
      }
    }
  }
  if (raw_triangles.size() != expected_triangles ||
      raw_triangle_columns.size() != raw_triangles.size()) {
    return fail("mesh_zipper_triangle_count_mismatch");
  }

  std::array<const SensorCloud*, 256> sensor_by_id{};
  for (const SensorCloud& sensor : sensors) {
    sensor_by_id[sensor.sensor_id] = &sensor;
  }
  struct EdgeRecord {
    std::size_t count = 0U;
    int directed_balance = 0;
  };
  const auto materialize_and_validate =
      [&vertex_sensor_ids, &sensor_by_id, maximum_edge_with_tolerance](
          SupportedStripMesh& output_mesh,
          const RawMeshTriangles& active_triangles,
          const std::vector<MeshColumnKey>& active_columns,
          MeshColumnSet& invalid_columns, std::string& reason) {
        output_mesh.triangles.clear();
        output_mesh.triangles.reserve(active_triangles.size());
        for (const RawMeshTriangle& raw : active_triangles) {
          if (raw[0] == raw[1] || raw[1] == raw[2] || raw[2] == raw[0] ||
              raw[0] >= output_mesh.vertices.size() ||
              raw[1] >= output_mesh.vertices.size() ||
              raw[2] >= output_mesh.vertices.size()) {
            reason = "mesh_zipper_degenerate_topology";
            return false;
          }
          const std::uint8_t sensor_id = vertex_sensor_ids[raw[0]];
          if (vertex_sensor_ids[raw[1]] != sensor_id ||
              vertex_sensor_ids[raw[2]] != sensor_id) {
            reason = "mesh_cross_sensor_triangle";
            return false;
          }
          const SensorCloud* sensor = sensor_by_id[sensor_id];
          if (sensor == nullptr) {
            reason = "mesh_missing_sensor_origin";
            return false;
          }
          Eigen::Vector3i triangle(static_cast<int>(raw[0]),
                                   static_cast<int>(raw[1]),
                                   static_cast<int>(raw[2]));
          const Eigen::Vector3f& a = output_mesh.vertices[raw[0]];
          const Eigen::Vector3f& b = output_mesh.vertices[raw[1]];
          const Eigen::Vector3f& c = output_mesh.vertices[raw[2]];
          const Eigen::Vector3f normal = (b - a).cross(c - a);
          if (!finitePoint(normal) || normal.squaredNorm() <= 1e-16f) {
            reason = "mesh_degenerate_triangle";
            return false;
          }
          const Eigen::Vector3f centroid = (a + b + c) / 3.0f;
          if (normal.dot(sensor->origin - centroid) < 0.0f) {
            std::swap(triangle.y(), triangle.z());
          }
          canonicalRotate(triangle);
          output_mesh.triangles.push_back(triangle);
        }
        std::sort(output_mesh.triangles.begin(), output_mesh.triangles.end(),
                  [](const Eigen::Vector3i& left,
                     const Eigen::Vector3i& right) {
                    return std::tie(left.x(), left.y(), left.z()) <
                           std::tie(right.x(), right.y(), right.z());
                  });
        const auto duplicate = std::adjacent_find(
            output_mesh.triangles.begin(), output_mesh.triangles.end(),
            [](const Eigen::Vector3i& left, const Eigen::Vector3i& right) {
              return left.x() == right.x() && left.y() == right.y() &&
                     left.z() == right.z();
            });
        if (duplicate != output_mesh.triangles.end()) {
          reason = "mesh_duplicate_triangle";
          return false;
        }

        std::unordered_map<IndexedMeshEdge, EdgeRecord,
                           IndexedMeshEdgeHash> edge_records;
        if (output_mesh.triangles.size() <=
            std::numeric_limits<std::size_t>::max() / 3U) {
          edge_records.reserve(output_mesh.triangles.size() * 3U);
        }
        for (const Eigen::Vector3i& triangle : output_mesh.triangles) {
          const std::array<std::size_t, 3> vertices =
              {{static_cast<std::size_t>(triangle.x()),
                static_cast<std::size_t>(triangle.y()),
                static_cast<std::size_t>(triangle.z())}};
          for (std::size_t edge_index = 0U; edge_index < 3U; ++edge_index) {
            const std::size_t from = vertices[edge_index];
            const std::size_t to = vertices[(edge_index + 1U) % 3U];
            if ((output_mesh.vertices[from] - output_mesh.vertices[to])
                    .norm() >
                maximum_edge_with_tolerance) {
              reason = "mesh_edge_limit_violation";
              return false;
            }
            EdgeRecord& record = edge_records[indexedMeshEdge(from, to)];
            ++record.count;
            record.directed_balance += from < to ? 1 : -1;
            if (record.count > 2U) {
              reason = "mesh_inconsistent_winding";
              return false;
            }
          }
        }
        for (const auto& edge : edge_records) {
          if (edge.second.count == 2U &&
              edge.second.directed_balance != 0) {
            reason = "mesh_inconsistent_winding";
            return false;
          }
        }
        std::vector<std::size_t> invalid_link_vertices;
        if (!vertexLinksAreSinglePathsOrRings(
                output_mesh.triangles, output_mesh.vertices.size(),
                &invalid_link_vertices)) {
          std::vector<std::uint8_t> invalid(output_mesh.vertices.size(), 0U);
          for (const std::size_t vertex : invalid_link_vertices) {
            if (vertex < invalid.size()) {
              invalid[vertex] = 1U;
            }
          }
          for (std::size_t triangle = 0U;
               triangle < active_triangles.size(); ++triangle) {
            const RawMeshTriangle& vertices = active_triangles[triangle];
            if ((invalid[vertices[0]] != 0U ||
                 invalid[vertices[1]] != 0U ||
                 invalid[vertices[2]] != 0U) &&
                triangle < active_columns.size()) {
              invalid_columns.insert(active_columns[triangle]);
            }
          }
          reason = "mesh_nonmanifold_vertex_link";
          return false;
        }
        return true;
      };

  const auto finalize_mesh =
      [&materialize_and_validate, &raw_triangles](
          const SupportedStripMesh& source_mesh,
          const RawMeshTriangles& active_triangles,
          const std::vector<MeshColumnKey>& active_columns,
          SupportedStripMesh& finalized_mesh,
          MeshColumnSet& exact_conflict_columns, std::string& reason) {
        exact_conflict_columns.clear();
        reason.clear();
        finalized_mesh = source_mesh;
        if (!materialize_and_validate(finalized_mesh, active_triangles,
                                      active_columns, exact_conflict_columns,
                                      reason)) {
          return MeshFinalizeStatus::kExactTopologyInvalid;
        }
        finalized_mesh.output_equivalence_removed_triangles =
            raw_triangles.size() - active_triangles.size();

        std::vector<std::uint8_t> vertex_used(
            finalized_mesh.vertices.size(), 0U);
        for (const Eigen::Vector3i& triangle : finalized_mesh.triangles) {
          for (int corner = 0; corner < 3; ++corner) {
            vertex_used[static_cast<std::size_t>(triangle[corner])] = 1U;
          }
        }
        std::vector<int> remap(finalized_mesh.vertices.size(), -1);
        std::vector<Eigen::Vector3f,
                    Eigen::aligned_allocator<Eigen::Vector3f>>
            compact_vertices;
        compact_vertices.reserve(finalized_mesh.vertices.size());
        for (std::size_t vertex = 0U;
             vertex < finalized_mesh.vertices.size(); ++vertex) {
          if (vertex_used[vertex] != 0U) {
            remap[vertex] = static_cast<int>(compact_vertices.size());
            compact_vertices.push_back(finalized_mesh.vertices[vertex]);
          }
        }
        for (Eigen::Vector3i& triangle : finalized_mesh.triangles) {
          for (int corner = 0; corner < 3; ++corner) {
            triangle[corner] =
                remap[static_cast<std::size_t>(triangle[corner])];
          }
        }
        finalized_mesh.vertices.swap(compact_vertices);
        if (!validateOutputTopology(finalized_mesh).valid) {
          reason = "mesh_compacted_output_topology_invalid";
          return MeshFinalizeStatus::kOutputTopologyInvalid;
        }
        return MeshFinalizeStatus::kAccepted;
      };

  MeshColumnSet output_filtered_columns;
  mesh.output_equivalence_input_triangles = raw_triangles.size();
  RawMeshTriangles active_triangles;
  std::vector<MeshColumnKey> active_columns;
  SupportedStripMesh repaired_mesh;
  bool output_mesh_finalized = false;
  std::string repair_reason;
  if (!repairOutputTopologyColumns(
          mesh, raw_triangles, raw_triangle_columns, vertex_sensor_ids,
          sensors, plans, finalize_mesh, output_filtered_columns,
          active_triangles, active_columns, repaired_mesh,
          output_mesh_finalized, repair_reason)) {
    return fail(repair_reason.c_str());
  }
  if (output_mesh_finalized) {
    mesh = std::move(repaired_mesh);
  }

  if (!output_mesh_finalized) {
    SupportedStripMesh finalized_mesh;
    MeshColumnSet exact_conflict_columns;
    std::string finalize_reason;
    const MeshFinalizeStatus finalize_status = finalize_mesh(
        mesh, active_triangles, active_columns, finalized_mesh,
        exact_conflict_columns, finalize_reason);
    if (finalize_status != MeshFinalizeStatus::kAccepted) {
      if (finalize_status == MeshFinalizeStatus::kExactTopologyInvalid &&
          nonmanifold_columns != nullptr) {
        nonmanifold_columns->insert(output_filtered_columns.begin(),
                                    output_filtered_columns.end());
        nonmanifold_columns->insert(exact_conflict_columns.begin(),
                                    exact_conflict_columns.end());
      }
      return fail(finalize_reason.c_str());
    }
    mesh = std::move(finalized_mesh);
  }
  if (nonmanifold_columns != nullptr) {
    nonmanifold_columns->insert(output_filtered_columns.begin(),
                                output_filtered_columns.end());
  }

  mesh.accepted = true;
  mesh.budget_limited = false;
  if (mesh.skipped_curve_intervals > 0U &&
      mesh.skipped_degenerate_intervals > 0U) {
    mesh.reason = "accepted_with_skipped_intervals";
  } else if (mesh.skipped_curve_intervals > 0U) {
    mesh.reason = "accepted_with_ring_curve_gaps";
  } else if (mesh.skipped_degenerate_intervals > 0U) {
    mesh.reason = "accepted_with_degenerate_intervals";
  } else {
    mesh.reason = "accepted";
  }
  return mesh;
}

SupportedStripMesh buildSupportedStripMesh(
    const SensorClouds& sensors, const std::vector<RasterWorks>& raster_work,
    const ScanStripSupportConfig& config) {
  SupportedStripMesh mesh;
  mesh.reason = "not_run";
  const double maximum_edge =
      static_cast<double>(config.voxel_size) *
      static_cast<double>(config.maximum_mesh_edge_voxels);
  const double weld_tolerance =
      static_cast<double>(config.voxel_size) *
      static_cast<double>(config.mesh_weld_tolerance_voxels);
  const double breakpoint_merge_tolerance = std::max(
      kDuplicateAzimuthEpsilon,
      weld_tolerance / static_cast<double>(config.max_range));
  if (!std::isfinite(maximum_edge) || !(maximum_edge > 0.0) ||
      !std::isfinite(weld_tolerance) || !(weld_tolerance > 0.0) ||
      !std::isfinite(breakpoint_merge_tolerance) ||
      !(breakpoint_merge_tolerance > 0.0)) {
    mesh.reason = "invalid_mesh_scale";
    return mesh;
  }

  std::array<bool, 256> seen_sensor_ids{};
  for (const SensorCloud& sensor : sensors) {
    if (seen_sensor_ids[sensor.sensor_id]) {
      mesh.reason = "duplicate_sensor_id";
      return mesh;
    }
    seen_sensor_ids[sensor.sensor_id] = true;
  }

  std::vector<SensorAtomicMeshData> atomic_sensors;
  for (std::size_t sensor_index = 0; sensor_index < raster_work.size();
       ++sensor_index) {
    std::map<std::uint16_t, MeshRingSamples> ring_curves;
    std::map<std::uint16_t, QuadSupportComponents> pair_components;
    std::map<std::uint16_t, bool> component_closed;
    std::vector<double> breakpoints;
    for (const RasterWork& work : raster_work[sensor_index]) {
      const QuadCandidate& quad = work.quad;
      QuadSupportComponents& components = pair_components[quad.lower_ring];
      if (components.empty() || component_closed[quad.lower_ring]) {
        components.emplace_back();
      }
      components.back().push_back(&quad);
      component_closed[quad.lower_ring] = work.include_end;
      ring_curves[quad.lower_ring].push_back(
          MeshRingSample{quad.lower_start_azimuth, quad.lower_start});
      ring_curves[quad.lower_ring].push_back(
          MeshRingSample{quad.lower_end_azimuth, quad.lower_end});
      ring_curves[static_cast<std::uint16_t>(quad.lower_ring + 1U)].push_back(
          MeshRingSample{quad.upper_start_azimuth, quad.upper_start});
      ring_curves[static_cast<std::uint16_t>(quad.lower_ring + 1U)].push_back(
          MeshRingSample{quad.upper_end_azimuth, quad.upper_end});
      breakpoints.push_back(normalizeAzimuth(quad.start_azimuth));
      breakpoints.push_back(
          normalizeAzimuth(quad.start_azimuth + quad.azimuth_width));
      breakpoints.push_back(normalizeAzimuth(quad.lower_start_azimuth));
      breakpoints.push_back(normalizeAzimuth(quad.lower_end_azimuth));
      breakpoints.push_back(normalizeAzimuth(quad.upper_start_azimuth));
      breakpoints.push_back(normalizeAzimuth(quad.upper_end_azimuth));
    }
    if (pair_components.empty()) {
      continue;
    }
    for (auto& curve : ring_curves) {
      canonicalizeRingSamples(curve.second);
    }
    const double maximum_curve_gap =
        radians(config.maximum_gap_angle_deg +
                2.0f * config.maximum_match_angle_deg);
    const double maximum_interval_width =
        radians(config.maximum_gap_angle_deg);
    std::sort(breakpoints.begin(), breakpoints.end());
    std::vector<double> canonical_breakpoints;
    canonical_breakpoints.reserve(breakpoints.size());
    for (const double breakpoint : breakpoints) {
      if (canonical_breakpoints.empty() ||
          breakpoint - canonical_breakpoints.back() >
              breakpoint_merge_tolerance ||
          !nearbyBreakpointsTopologyCompatible(
              canonical_breakpoints.back(), breakpoint, ring_curves,
              maximum_curve_gap, weld_tolerance)) {
        canonical_breakpoints.push_back(breakpoint);
      }
    }
    breakpoints.swap(canonical_breakpoints);
    if (breakpoints.size() > 1U &&
        forwardAngle(breakpoints.back(), breakpoints.front()) <=
            breakpoint_merge_tolerance &&
        nearbyBreakpointsTopologyCompatible(
            breakpoints.back(), breakpoints.front(), ring_curves,
            maximum_curve_gap, weld_tolerance)) {
      breakpoints.pop_back();
    }
    const double target_along_edge = 0.5 * maximum_edge;
    if (!std::isfinite(target_along_edge) || !(target_along_edge > 0.0)) {
      mesh.reason = "invalid_mesh_scale";
      return mesh;
    }
    std::vector<double> refined_breakpoints;
    refined_breakpoints.reserve(breakpoints.size());
    for (std::size_t index = 0U; index < breakpoints.size(); ++index) {
      const double start = breakpoints[index];
      const double end = breakpoints[(index + 1U) % breakpoints.size()];
      const double width = forwardAngle(start, end);
      std::size_t subdivisions = 1U;
      if (width > kDuplicateAzimuthEpsilon &&
          width <= maximum_interval_width) {
        double maximum_ring_edge = 0.0;
        for (const auto& curve : ring_curves) {
          Eigen::Vector3f start_position = Eigen::Vector3f::Zero();
          Eigen::Vector3f end_position = Eigen::Vector3f::Zero();
          if (!sampleRingCurve(curve.second, start, maximum_curve_gap,
                               start_position) ||
              !sampleRingCurve(curve.second, end, maximum_curve_gap,
                               end_position)) {
            continue;
          }
          const double ring_edge =
              static_cast<double>((end_position - start_position).norm());
          if (!std::isfinite(ring_edge)) {
            mesh.reason = "mesh_nonfinite_ring_curve";
            return mesh;
          }
          maximum_ring_edge = std::max(maximum_ring_edge, ring_edge);
        }
        const double subdivision_value =
            maximum_ring_edge / target_along_edge;
        if (!std::isfinite(subdivision_value) ||
            subdivision_value >
                static_cast<double>(config.max_candidate_samples)) {
          mesh.budget_limited = true;
          mesh.reason = "mesh_budget_limited";
          return mesh;
        }
        subdivisions = std::max<std::size_t>(
            1U, static_cast<std::size_t>(std::ceil(subdivision_value)));
      }
      if (refined_breakpoints.size() > config.max_candidate_samples ||
          subdivisions >
              config.max_candidate_samples - refined_breakpoints.size()) {
        mesh.budget_limited = true;
        mesh.reason = "mesh_budget_limited";
        return mesh;
      }
      for (std::size_t step = 0U; step < subdivisions; ++step) {
        const double fraction = static_cast<double>(step) /
                                static_cast<double>(subdivisions);
        const double breakpoint = normalizeAzimuth(start + fraction * width);
        if (!std::isfinite(breakpoint)) {
          mesh.reason = "mesh_nonfinite_breakpoint";
          return mesh;
        }
        refined_breakpoints.push_back(breakpoint);
      }
    }
    breakpoints.swap(refined_breakpoints);
    std::vector<MeshBreakpointInterval> intervals(breakpoints.size());
    std::vector<IndexedAzimuth> sorted_midpoints;
    sorted_midpoints.reserve(breakpoints.size());
    for (std::size_t index = 0; index < breakpoints.size(); ++index) {
      MeshBreakpointInterval& interval = intervals[index];
      interval.start = breakpoints[index];
      interval.end = breakpoints[(index + 1U) % breakpoints.size()];
      interval.width = forwardAngle(interval.start, interval.end);
      interval.usable = interval.width > kDuplicateAzimuthEpsilon &&
                        interval.width <= maximum_interval_width;
      interval.midpoint =
          normalizeAzimuth(interval.start + 0.5 * interval.width);
      sorted_midpoints.emplace_back(interval.midpoint, index);
    }
    std::sort(sorted_midpoints.begin(), sorted_midpoints.end());
    std::map<std::uint16_t, CachedRingCurveSamples> curve_sample_cache;
    const auto sample_cached =
        [maximum_curve_gap](const MeshRingSamples& curve,
                            CachedRingCurveSamples& cache,
                            std::size_t breakpoint_index,
                            const std::vector<double>& values,
                            Eigen::Vector3f& position) {
          if (cache.empty()) {
            cache.resize(values.size());
          }
          CachedRingCurveSample& sample = cache[breakpoint_index];
          if (!sample.computed) {
            sample.valid = sampleRingCurve(
                curve, values[breakpoint_index], maximum_curve_gap,
                sample.position);
            sample.computed = true;
          }
          if (sample.valid) {
            position = sample.position;
          }
          return sample.valid;
        };
    SensorAtomicMeshData sensor_data;
    sensor_data.sensor_index = sensor_index;
    sensor_data.sensor_id = sensors[sensor_index].sensor_id;
    sensor_data.columns.resize(intervals.size());
    for (std::size_t index = 0U; index < intervals.size(); ++index) {
      AtomicMeshColumn& column = sensor_data.columns[index];
      column.start_breakpoint = index;
      column.end_breakpoint = (index + 1U) % intervals.size();
      column.start_azimuth = intervals[index].start;
      column.end_azimuth = intervals[index].end;
      column.width = intervals[index].width;
      column.usable = intervals[index].usable;
    }
    for (const auto& pair : pair_components) {
      if (pair.first == std::numeric_limits<std::uint16_t>::max()) {
        continue;
      }
      const auto lower_curve = ring_curves.find(pair.first);
      const auto upper_curve = ring_curves.find(
          static_cast<std::uint16_t>(pair.first + 1U));
      if (lower_curve == ring_curves.end() ||
          upper_curve == ring_curves.end()) {
        mesh.reason = "mesh_missing_ring_curve";
        return mesh;
      }
      CachedRingCurveSamples& lower_cache = curve_sample_cache[pair.first];
      CachedRingCurveSamples& upper_cache = curve_sample_cache[
          static_cast<std::uint16_t>(pair.first + 1U)];
      const std::vector<std::int64_t> covered = coveredBreakpointComponents(
          pair.second, sorted_midpoints, breakpoints.size());
      for (std::size_t index = 0; index < breakpoints.size(); ++index) {
        const MeshBreakpointInterval& interval = intervals[index];
        AtomicMeshColumn& column = sensor_data.columns[index];
        if (!interval.usable || covered[index] == kNoSupportComponent) {
          continue;
        }
        column.had_support = true;
        if (covered[index] == kAmbiguousSupportComponent) {
          column.hard_cut = true;
          continue;
        }
        MeshQuadPlan plan;
        plan.sensor_id = sensors[sensor_index].sensor_id;
        plan.quad.lower_ring = pair.first;
        plan.quad.start_azimuth = interval.start;
        plan.quad.end_azimuth = interval.end;
        plan.quad.azimuth_width = interval.width;
        plan.quad.midpoint_azimuth = interval.midpoint;
        plan.quad.lower_start_azimuth = interval.start;
        plan.quad.lower_end_azimuth = interval.end;
        plan.quad.upper_start_azimuth = interval.start;
        plan.quad.upper_end_azimuth = interval.end;
        const std::size_t end_index =
            (index + 1U) % breakpoints.size();
        plan.start_breakpoint = index;
        plan.end_breakpoint = end_index;
        const bool lower_start_valid = sample_cached(
            lower_curve->second, lower_cache, index, breakpoints,
            plan.quad.lower_start);
        const bool lower_end_valid = sample_cached(
            lower_curve->second, lower_cache, end_index, breakpoints,
            plan.quad.lower_end);
        const bool upper_start_valid = sample_cached(
            upper_curve->second, upper_cache, index, breakpoints,
            plan.quad.upper_start);
        const bool upper_end_valid = sample_cached(
            upper_curve->second, upper_cache, end_index, breakpoints,
            plan.quad.upper_end);
        if (!lower_start_valid || !lower_end_valid || !upper_start_valid ||
            !upper_end_valid) {
          ++mesh.skipped_curve_intervals;
          column.hard_cut = true;
          if (!mesh.has_first_curve_gap) {
            mesh.has_first_curve_gap = true;
            mesh.first_curve_gap_sensor_id = sensors[sensor_index].sensor_id;
            mesh.first_curve_gap_lower_ring = pair.first;
            mesh.first_curve_gap_start_azimuth = interval.start;
            mesh.first_curve_gap_end_azimuth = interval.end;
            mesh.first_curve_gap_corner =
                !lower_start_valid ? "lower_start"
                : !lower_end_valid ? "lower_end"
                : !upper_start_valid ? "upper_start"
                                     : "upper_end";
          }
          continue;
        }
        const double atomic_edge_tolerance = 1e-5 * target_along_edge;
        if (!populateMeshPlanGeometry(sensors[sensor_index], config, plan) ||
            static_cast<double>(plan.quad.maximum_along_edge) >
                target_along_edge + atomic_edge_tolerance) {
          ++mesh.skipped_degenerate_intervals;
          column.hard_cut = true;
          continue;
        }
        const MeshPlanCornerStatus corner_status =
            populateMeshPlanCorners(plan, weld_tolerance);
        if (corner_status == MeshPlanCornerStatus::kOverflow) {
          mesh.reason = "mesh_coordinate_overflow";
          return mesh;
        }
        if (corner_status == MeshPlanCornerStatus::kDegenerate) {
          ++mesh.skipped_degenerate_intervals;
          column.hard_cut = true;
          continue;
        }
        column.signature.push_back(MeshSupportComponent{
            pair.first, static_cast<std::size_t>(covered[index])});
        column.plans.push_back(std::move(plan));
      }
    }
    for (AtomicMeshColumn& column : sensor_data.columns) {
      if (column.hard_cut) {
        if (column.had_support) {
          ++mesh.skipped_sensor_columns;
        }
        column.signature.clear();
        column.plans.clear();
      }
    }
    atomic_sensors.push_back(std::move(sensor_data));
  }
  std::size_t maximum_mask_iterations = 0U;
  for (const SensorAtomicMeshData& sensor : atomic_sensors) {
    maximum_mask_iterations += sensor.columns.size();
  }
  MeshColumnSet masked_atomic_columns;
  for (std::size_t iteration = 0U; iteration <= maximum_mask_iterations;
       ++iteration) {
    MeshQuadPlans plans;
    MergedAtomicColumns merged_atomic_columns;
    for (const SensorAtomicMeshData& sensor_data : atomic_sensors) {
      if (sensor_data.sensor_index >= sensors.size()) {
        mesh.reason = "mesh_missing_sensor";
        return mesh;
      }
      appendCoarsenedSensorPlans(
          sensor_data, sensors[sensor_data.sensor_index],
          masked_atomic_columns, config, 0.5 * maximum_edge,
          weld_tolerance, plans, merged_atomic_columns);
    }
    std::sort(plans.begin(), plans.end(),
              [](const MeshQuadPlan& left, const MeshQuadPlan& right) {
                if (left.sensor_id != right.sensor_id) {
                  return left.sensor_id < right.sensor_id;
                }
                if (left.quad.lower_ring != right.quad.lower_ring) {
                  return left.quad.lower_ring < right.quad.lower_ring;
                }
                if (left.start_breakpoint != right.start_breakpoint) {
                  return left.start_breakpoint < right.start_breakpoint;
                }
                if (left.end_breakpoint != right.end_breakpoint) {
                  return left.end_breakpoint < right.end_breakpoint;
                }
                if (vectorLexicographicallyLess(left.quad.lower_start,
                                                right.quad.lower_start)) {
                  return true;
                }
                if (vectorLexicographicallyLess(right.quad.lower_start,
                                                left.quad.lower_start)) {
                  return false;
                }
                return vectorLexicographicallyLess(left.quad.upper_end,
                                                   right.quad.upper_end);
              });
    mesh.curve_intervals = plans.size();
    if (plans.empty()) {
      mesh.accepted = true;
      if (!masked_atomic_columns.empty()) {
        mesh.reason = "no_manifold_strips";
      } else if (mesh.skipped_degenerate_intervals > 0U) {
        mesh.reason = "no_nondegenerate_strips";
      } else if (mesh.skipped_curve_intervals > 0U) {
        mesh.reason = "no_resampleable_strips";
      } else {
        mesh.reason = "no_supported_strips";
      }
      return mesh;
    }
    MeshColumnSet nonmanifold_columns;
    SupportedStripMesh candidate = buildZipperStripMesh(
        sensors, plans, config, maximum_edge, weld_tolerance, mesh,
        &nonmanifold_columns);
    if (candidate.accepted && !nonmanifold_columns.empty()) {
      MeshColumnSet output_atomic_columns;
      for (const MeshColumnKey& merged : nonmanifold_columns) {
        const auto found = merged_atomic_columns.find(merged);
        if (found == merged_atomic_columns.end()) {
          candidate.accepted = false;
          candidate.reason = "mesh_output_column_mapping_missing";
          candidate.vertices.clear();
          candidate.triangles.clear();
          return candidate;
        }
        output_atomic_columns.insert(found->second.begin(),
                                     found->second.end());
      }
      std::size_t filtered_plans = 0U;
      for (const MeshQuadPlan& plan : plans) {
        filtered_plans +=
            nonmanifold_columns.count(meshColumnKey(plan)) != 0U ? 1U : 0U;
      }
      if (filtered_plans > candidate.curve_intervals) {
        candidate.accepted = false;
        candidate.reason = "mesh_output_column_count_mismatch";
        candidate.vertices.clear();
        candidate.triangles.clear();
        return candidate;
      }
      candidate.curve_intervals -= filtered_plans;
      candidate.skipped_sensor_columns += output_atomic_columns.size();
      candidate.output_equivalence_masked_sensor_columns =
          output_atomic_columns.size();
      if (candidate.skipped_curve_intervals == 0U &&
          candidate.skipped_degenerate_intervals == 0U) {
        candidate.reason = "accepted_with_output_equivalence_columns";
      }
      return candidate;
    }
    if ((candidate.reason != "mesh_nonmanifold_vertex_link" &&
         candidate.reason != "mesh_output_equivalence_conflict") ||
        nonmanifold_columns.empty()) {
      return candidate;
    }
    std::size_t added_masks = 0U;
    for (const MeshColumnKey& merged : nonmanifold_columns) {
      const auto found = merged_atomic_columns.find(merged);
      if (found == merged_atomic_columns.end()) {
        return candidate;
      }
      for (const MeshColumnKey& atomic : found->second) {
        if (masked_atomic_columns.insert(atomic).second) {
          ++added_masks;
        }
      }
    }
    if (added_masks == 0U) {
      return candidate;
    }
    mesh.skipped_sensor_columns += added_masks;
  }
  mesh.reason = "mesh_column_mask_iteration_limit";
  return mesh;

}

}  // namespace

bool assignSourceTopologyFields(bool has_ring, double raw_ring,
                                bool has_azimuth, double raw_azimuth,
                                TsdfPoint& point) {
  point.ring = 0U;
  point.azimuth = std::numeric_limits<float>::quiet_NaN();
  point.source_topology_valid = false;
  const bool ring_valid =
      has_ring && std::isfinite(raw_ring) && raw_ring >= 0.0 &&
      raw_ring <= 65535.0 && std::floor(raw_ring) == raw_ring;
  const bool azimuth_valid =
      has_azimuth && std::isfinite(raw_azimuth) &&
      std::abs(raw_azimuth) <= kTwoPi + 1e-3;
  if (ring_valid) {
    point.ring = static_cast<std::uint16_t>(raw_ring);
  }
  if (azimuth_valid) {
    point.azimuth = static_cast<float>(raw_azimuth);
  }
  point.source_topology_valid = ring_valid && azimuth_valid;
  return point.source_topology_valid;
}

ScanStripSupportResult buildSupportedScanStrips(
    const SensorClouds& sensors, const ScanStripSupportConfig& config) {
  ScanStripSupportResult result;
  if (!validConfiguration(config)) {
    result.reason = "invalid_configuration";
    return result;
  }
  result.accepted = true;
  if (!config.enabled) {
    result.reason = "disabled";
    return result;
  }

  std::vector<RasterWorks> raster_work(sensors.size());
  for (std::size_t sensor_index = 0; sensor_index < sensors.size();
       ++sensor_index) {
    const SensorCloud& sensor = sensors[sensor_index];
    std::map<std::uint16_t, RingScan> scans =
        buildRingScans(sensor, config, result.stats);
    std::map<std::uint16_t, PairStrip> strips;
    for (auto lower_it = scans.begin(); lower_it != scans.end(); ++lower_it) {
      if (lower_it->first == std::numeric_limits<std::uint16_t>::max()) {
        continue;
      }
      const auto upper_it = scans.find(
          static_cast<std::uint16_t>(lower_it->first + 1U));
      if (upper_it == scans.end()) {
        continue;
      }
      ++result.stats.ring_pairs;
      const std::vector<MatchedRung> rungs =
          matchRungs(lower_it->second, upper_it->second, config);
      const double ring_angle = medianRingAngle(
          lower_it->second, upper_it->second, rungs, sensor.origin);
      if (rungs.size() < 3U || !std::isfinite(ring_angle) ||
          ring_angle < radians(config.minimum_ring_angle_deg) ||
          ring_angle > radians(config.maximum_ring_angle_deg)) {
        ++result.stats.rejected_ring_pairs;
        continue;
      }
      PairStrip strip;
      strip.lower_ring = lower_it->first;
      strip.ring_step_direction = ringStepDirection(
          lower_it->second, upper_it->second, rungs, sensor.origin);
      strip.quads.reserve(rungs.size() - 1U);
      for (std::size_t index = 0; index + 1U < rungs.size(); ++index) {
        QuadCandidate candidate = evaluateQuad(
            lower_it->second, upper_it->second, rungs[index],
            rungs[index + 1U], config);
        ++result.stats.candidate_quads;
        if (candidate.locally_valid) {
          ++result.stats.locally_valid_quads;
          if (candidate.strong) {
            ++result.stats.strong_quads;
          }
        }
        strip.quads.push_back(std::move(candidate));
      }
      buildAzimuthIndex(strip);
      strips.emplace(strip.lower_ring, std::move(strip));
    }

    // Numeric ring adjacency is accepted only when at least two consecutive
    // ring pairs advance in the same median ray-elevation direction.  This is
    // invariant to the sensor extrinsic rotation and rejects scrambled ring
    // IDs without needing a hard-coded LiDAR vertical axis.  With fewer than
    // three observable rings the augmentation fails closed.
    const float ring_order_cosine = static_cast<float>(std::cos(
        radians(config.maximum_ring_order_direction_angle_deg)));
    for (auto& strip_item : strips) {
      PairStrip& strip = strip_item.second;
      const auto previous =
          strip.lower_ring == 0U
              ? strips.end()
              : strips.find(static_cast<std::uint16_t>(strip.lower_ring - 1U));
      const auto next =
          strip.lower_ring == std::numeric_limits<std::uint16_t>::max()
              ? strips.end()
              : strips.find(static_cast<std::uint16_t>(strip.lower_ring + 1U));
      strip.ring_order_valid =
          strip.ring_step_direction.squaredNorm() > 0.0f &&
          ((previous != strips.end() &&
            strip.ring_step_direction.dot(
                previous->second.ring_step_direction) >= ring_order_cosine) ||
           (next != strips.end() &&
            strip.ring_step_direction.dot(next->second.ring_step_direction) >=
                ring_order_cosine));
      if (!strip.ring_order_valid) {
        ++result.stats.rejected_ring_order_pairs;
        for (QuadCandidate& candidate : strip.quads) {
          candidate.locally_valid = false;
          candidate.strong = false;
        }
      }
    }

    // A long span is allowed only when a third, adjacent physical ring lies on
    // the same plane.  This rejects coherent-looking depth curtains at wall and
    // occlusion transitions without penalising genuinely grazing floor/roof.
    for (auto& strip_item : strips) {
      PairStrip& strip = strip_item.second;
      const auto previous =
          strip.lower_ring == 0U
              ? strips.end()
              : strips.find(static_cast<std::uint16_t>(strip.lower_ring - 1U));
      const auto next =
          strip.lower_ring == std::numeric_limits<std::uint16_t>::max()
              ? strips.end()
              : strips.find(static_cast<std::uint16_t>(strip.lower_ring + 1U));
      for (QuadCandidate& candidate : strip.quads) {
        if (!candidate.locally_valid) {
          continue;
        }
        if (candidate.strong) {
          candidate.final_valid = true;
          continue;
        }
        const bool supported =
            (previous != strips.end() &&
             neighborSupportsLongQuad(candidate, previous->second, config)) ||
            (next != strips.end() &&
             neighborSupportsLongQuad(candidate, next->second, config));
        if (supported) {
          candidate.verified_long = true;
          candidate.final_valid = true;
          ++result.stats.verified_long_quads;
        } else {
          ++result.stats.rejected_long_quads;
        }
      }
    }

    const float run_normal_cosine = static_cast<float>(
        std::cos(radians(config.maximum_quad_normal_angle_deg)));
    for (auto& strip_item : strips) {
      PairStrip& strip = strip_item.second;
      std::size_t index = 0U;
      while (index < strip.quads.size()) {
        if (!strip.quads[index].final_valid) {
          ++index;
          continue;
        }
        std::size_t end = index + 1U;
        while (end < strip.quads.size() &&
               strip.quads[end].final_valid &&
               strip.quads[end - 1U].normal.dot(strip.quads[end].normal) >=
                   run_normal_cosine &&
               lengthRatio(strip.quads[end - 1U].maximum_cross_edge,
                           strip.quads[end].maximum_cross_edge) <=
                   config.maximum_run_cross_edge_ratio) {
          ++end;
        }
        const std::size_t run_length = end - index;
        if (run_length < config.minimum_run_quads) {
          result.stats.rejected_isolated_quads += run_length;
          index = end;
          continue;
        }
        result.stats.accepted_run_quads += run_length;
        for (std::size_t quad_index = index; quad_index < end; ++quad_index) {
          RasterWork work;
          work.quad = strip.quads[quad_index];
          work.include_end = quad_index + 1U == end;
          raster_work[sensor_index].push_back(std::move(work));
        }
        index = end;
      }
    }
  }

  if (config.build_indexed_mesh) {
    result.mesh = buildSupportedStripMesh(sensors, raster_work, config);
  }

  if (!config.build_surface_rays) {
    result.reason = result.stats.accepted_run_quads == 0U
                        ? "no_supported_strips"
                        : "surface_rays_disabled";
    return result;
  }

  std::vector<SurfaceCellMap> surface_cells(sensors.size());

  // Candidate work is scheduled as complete quads, using the same progressive
  // sensor/ring/azimuth ordering as measured rays.  This prevents an early,
  // dense LiDAR from consuming the entire cap and ensures the cap can never
  // leave a half-rasterized quad in the surface-cell map.
  SensorClouds work_clouds;
  work_clouds.resize(sensors.size());
  std::size_t work_count = 0U;
  for (std::size_t sensor_index = 0; sensor_index < sensors.size();
       ++sensor_index) {
    SensorCloud& work_cloud = work_clouds[sensor_index];
    work_cloud.sensor_id = sensors[sensor_index].sensor_id;
    work_cloud.origin = sensors[sensor_index].origin;
    work_cloud.points.reserve(raster_work[sensor_index].size());
    for (const RasterWork& work : raster_work[sensor_index]) {
      // Use a measured corner for range filtering (the quad centroid can lie
      // just inside min_range on a curved strip), while retaining the quad's
      // midpoint azimuth for fair sector scheduling.
      TsdfPoint representative(work.quad.lower_start);
      representative.ring = work.quad.lower_ring;
      representative.azimuth =
          static_cast<float>(work.quad.midpoint_azimuth);
      representative.source_topology_valid = true;
      work_cloud.points.push_back(representative);
      ++work_count;
    }
  }
  const std::vector<RaySelection> work_order = selectStratifiedRays(
      work_clouds, work_count, config.min_range, config.max_range);
  for (const RaySelection& selected : work_order) {
    if (selected.sensor_index >= raster_work.size() ||
        selected.point_index >= raster_work[selected.sensor_index].size()) {
      // The selector is internal and should preserve these indices.  Fail
      // closed if that contract is ever broken.
      result.candidate_budget_limited = true;
      continue;
    }
    const RasterWork& work =
        raster_work[selected.sensor_index][selected.point_index];
    std::size_t sample_count = 0U;
    if (!rasterSampleCount(work.quad, work.include_end, config,
                           sample_count)) {
      result.candidate_budget_limited = true;
      continue;
    }
    const std::size_t remaining =
        config.max_candidate_samples - result.stats.candidate_samples;
    if (sample_count > remaining) {
      result.candidate_budget_limited = true;
      continue;
    }
    if (!rasterizeQuad(selected.sensor_index, work.quad, work.include_end,
                       config, surface_cells, result)) {
      // rasterSampleCount has already validated every loop bound, so reaching
      // this branch indicates an internal contract violation.  Do not attempt
      // any further virtual work.
      result.candidate_budget_limited = true;
      break;
    }
  }

  SensorClouds candidate_clouds;
  candidate_clouds.resize(sensors.size());
  std::vector<std::vector<SurfaceCandidate>> candidate_metadata(sensors.size());
  for (std::size_t sensor_index = 0; sensor_index < sensors.size();
       ++sensor_index) {
    SensorCloud& candidates = candidate_clouds[sensor_index];
    candidates.sensor_id = sensors[sensor_index].sensor_id;
    candidates.origin = sensors[sensor_index].origin;
    candidates.points.reserve(surface_cells[sensor_index].size());
    candidate_metadata[sensor_index].reserve(surface_cells[sensor_index].size());
    for (const auto& cell : surface_cells[sensor_index]) {
      TsdfPoint point(cell.second.position);
      point.ring = cell.second.lower_ring;
      point.azimuth = cell.second.azimuth;
      point.source_topology_valid = true;
      candidates.points.push_back(point);
      candidate_metadata[sensor_index].push_back(cell.second);
    }
    result.stats.unique_surface_cells += candidates.points.size();
  }

  result.surface_budget_limited =
      result.stats.unique_surface_cells > config.max_surface_cells;
  const std::vector<RaySelection> selection = selectStratifiedRays(
      candidate_clouds, config.max_surface_cells, config.min_range,
      config.max_range);
  result.rays.reserve(selection.size());
  for (const RaySelection& selected : selection) {
    const SurfaceCandidate& candidate =
        candidate_metadata[selected.sensor_index][selected.point_index];
    SupportedSurfaceRay ray;
    ray.sensor_index = selected.sensor_index;
    ray.sensor_id = sensors[selected.sensor_index].sensor_id;
    ray.lower_ring = candidate.lower_ring;
    ray.azimuth = candidate.azimuth;
    ray.position = candidate.position;
    ray.verified_long_span = candidate.verified_long_span;
    result.rays.push_back(ray);
  }
  result.stats.selected_surface_rays = result.rays.size();
  if (result.candidate_budget_limited) {
    result.reason = "candidate_budget_limited";
  } else if (result.rays.empty()) {
    result.reason = "no_supported_strips";
  } else if (result.surface_budget_limited) {
    result.reason = "surface_budget_limited";
  } else {
    result.reason = "accepted";
  }
  return result;
}

}  // namespace local_tsdf_mesh
