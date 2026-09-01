#include "local_tsdf_mesh/frame_registrar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace local_tsdf_mesh {
namespace {

double rotationAngle(const Eigen::Matrix3f& rotation) {
  const double cosine = std::max(
      -1.0, std::min(1.0, (static_cast<double>(rotation.trace()) - 1.0) * 0.5));
  return std::acos(cosine);
}

double calculateInlierRatio(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
    const Eigen::Matrix4f& target_from_source,
    double inlier_distance) {
  if (!source || !target || source->empty() || target->empty() ||
      inlier_distance <= 0.0) {
    return 0.0;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*source, *aligned, target_from_source);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  const float max_distance_squared =
      static_cast<float>(inlier_distance * inlier_distance);
  std::vector<int> neighbor(1);
  std::vector<float> squared_distance(1);
  std::size_t valid = 0;
  std::size_t inliers = 0;
  for (const auto& point : aligned->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    ++valid;
    if (tree.nearestKSearch(point, 1, neighbor, squared_distance) == 1 &&
        squared_distance.front() <= max_distance_squared) {
      ++inliers;
    }
  }
  return valid > 0 ? static_cast<double>(inliers) / static_cast<double>(valid)
                   : 0.0;
}

}  // namespace

std::string registrationRejectReason(const RegistrationQuality& quality,
                                     const RegistrationConfig& config) {
  if (quality.source_points < config.min_points ||
      quality.target_points < config.min_points) {
    return "too_few_points";
  }
  if (!quality.converged) {
    return "not_converged";
  }
  if (!quality.transform_finite || !std::isfinite(quality.fitness_score) ||
      !std::isfinite(quality.inlier_ratio) ||
      !std::isfinite(quality.translation) ||
      !std::isfinite(quality.rotation_rad)) {
    return "nonfinite_result";
  }
  if (quality.fitness_score > config.max_fitness_score) {
    return "fitness_gate";
  }
  if (quality.inlier_ratio < config.min_inlier_ratio) {
    return "inlier_gate";
  }
  if (quality.translation > config.max_translation) {
    return "translation_gate";
  }
  if (quality.rotation_rad > config.max_rotation_rad) {
    return "rotation_gate";
  }
  return "accepted";
}

std::string motionConsistencyRejectReason(
    const Eigen::Matrix4f& candidate_target_from_source,
    const Eigen::Matrix4f& predicted_target_from_source,
    bool prediction_valid, const RegistrationConfig& config,
    double* translation_error, double* rotation_error_rad) {
  if (translation_error) {
    *translation_error = std::numeric_limits<double>::infinity();
  }
  if (rotation_error_rad) {
    *rotation_error_rad = std::numeric_limits<double>::infinity();
  }
  if (!candidate_target_from_source.allFinite() ||
      !predicted_target_from_source.allFinite()) {
    return "nonfinite_motion_prediction";
  }
  if (!prediction_valid) {
    const double translation = static_cast<double>(
        candidate_target_from_source.block<3, 1>(0, 3).norm());
    const double rotation =
        rotationAngle(candidate_target_from_source.block<3, 3>(0, 0));
    if (translation_error) {
      *translation_error = translation;
    }
    if (rotation_error_rad) {
      *rotation_error_rad = rotation;
    }
    // Fitness and inlier ratio do not make motion observable in a repetitive
    // tunnel. Without a prior, even a sizeable ICP translation can be a false
    // local minimum. The default therefore refuses all pure-ICP bootstraps.
    if (!config.allow_unverified_bootstrap) {
      return "degenerate_motion";
    }
    if (!config.allow_near_identity_bootstrap &&
        translation < config.minimum_bootstrap_translation &&
        rotation < config.minimum_bootstrap_rotation_rad) {
      return "identity_motion_bootstrap";
    }
    return "accepted";
  }

  const Eigen::Matrix4f prediction_from_candidate =
      predicted_target_from_source.inverse() * candidate_target_from_source;
  if (!prediction_from_candidate.allFinite()) {
    return "nonfinite_motion_prediction";
  }
  const double translation = static_cast<double>(
      prediction_from_candidate.block<3, 1>(0, 3).norm());
  const double rotation =
      rotationAngle(prediction_from_candidate.block<3, 3>(0, 0));
  if (translation_error) {
    *translation_error = translation;
  }
  if (rotation_error_rad) {
    *rotation_error_rad = rotation;
  }
  if (translation > config.max_prediction_translation_error) {
    return "motion_prediction_translation_gate";
  }
  if (rotation > config.max_prediction_rotation_error_rad) {
    return "motion_prediction_rotation_gate";
  }
  return "accepted";
}

FrameRegistrar::FrameRegistrar(const RegistrationConfig& config)
    : config_(config) {
  const bool finite =
      std::isfinite(config_.max_correspondence_distance) &&
      std::isfinite(config_.transformation_epsilon) &&
      std::isfinite(config_.rotation_epsilon) &&
      std::isfinite(config_.fitness_epsilon) &&
      std::isfinite(config_.max_fitness_score) &&
      std::isfinite(config_.inlier_distance) &&
      std::isfinite(config_.min_inlier_ratio) &&
      std::isfinite(config_.max_translation) &&
      std::isfinite(config_.max_rotation_rad) &&
      std::isfinite(config_.minimum_bootstrap_translation) &&
      std::isfinite(config_.minimum_bootstrap_rotation_rad) &&
      std::isfinite(config_.max_prediction_translation_error) &&
      std::isfinite(config_.max_prediction_rotation_error_rad);
  if (!finite || config_.maximum_iterations <= 0 ||
      config_.correspondence_randomness <= 0 || config_.min_points < 5 ||
      config_.max_correspondence_distance <= 0.0 ||
      config_.transformation_epsilon <= 0.0 ||
      config_.rotation_epsilon <= 0.0 || config_.fitness_epsilon <= 0.0 ||
      config_.max_fitness_score < 0.0 || config_.inlier_distance <= 0.0 ||
      config_.min_inlier_ratio < 0.0 || config_.min_inlier_ratio > 1.0 ||
      config_.max_translation < 0.0 || config_.max_rotation_rad < 0.0 ||
      config_.minimum_bootstrap_translation < 0.0 ||
      config_.minimum_bootstrap_rotation_rad < 0.0 ||
      config_.max_prediction_translation_error < 0.0 ||
      config_.max_prediction_rotation_error_rad < 0.0) {
    throw std::invalid_argument("invalid registration configuration");
  }
}

RegistrationResult FrameRegistrar::align(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
    const Eigen::Matrix4f& initial_guess) const {
  RegistrationResult result;
  result.quality.source_points = source ? source->size() : 0;
  result.quality.target_points = target ? target->size() : 0;
  if (!source || !target || result.quality.source_points < config_.min_points ||
      result.quality.target_points < config_.min_points) {
    result.reason = "too_few_points";
    return result;
  }
  if (!initial_guess.allFinite()) {
    result.reason = "nonfinite_initial_guess";
    return result;
  }

  try {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(std::max(1, config_.maximum_iterations));
    const std::size_t available_points =
        std::min(result.quality.source_points, result.quality.target_points);
    const int maximum_neighbors = static_cast<int>(std::min<std::size_t>(
        available_points, static_cast<std::size_t>(std::numeric_limits<int>::max())));
    gicp.setCorrespondenceRandomness(std::max(
        5, std::min(config_.correspondence_randomness, maximum_neighbors)));
    gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
    gicp.setTransformationEpsilon(config_.transformation_epsilon);
    gicp.setRotationEpsilon(config_.rotation_epsilon);
    gicp.setEuclideanFitnessEpsilon(config_.fitness_epsilon);
    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    gicp.align(aligned, initial_guess);
    result.quality.converged = gicp.hasConverged();
    result.target_from_source = gicp.getFinalTransformation();
    result.quality.transform_finite = result.target_from_source.allFinite();
    // PCL's implementation compares squared nearest-neighbor distances to this
    // argument even though the API calls it max_range.
    result.quality.fitness_score = gicp.getFitnessScore(
        config_.max_correspondence_distance *
        config_.max_correspondence_distance);
    if (result.quality.transform_finite) {
      result.quality.translation =
          static_cast<double>(result.target_from_source.block<3, 1>(0, 3).norm());
      result.quality.rotation_rad =
          rotationAngle(result.target_from_source.block<3, 3>(0, 0));
      result.quality.inlier_ratio =
          calculateInlierRatio(source, target, result.target_from_source,
                               config_.inlier_distance);
    } else {
      result.quality.fitness_score = std::numeric_limits<double>::infinity();
    }
  } catch (const std::exception&) {
    result.reason = "gicp_exception";
    return result;
  } catch (...) {
    result.reason = "gicp_exception";
    return result;
  }

  result.reason = registrationRejectReason(result.quality, config_);
  result.accepted = result.reason == "accepted";
  return result;
}

}  // namespace local_tsdf_mesh
