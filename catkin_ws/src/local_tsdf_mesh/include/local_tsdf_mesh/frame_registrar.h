#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace local_tsdf_mesh {

struct RegistrationConfig {
  int maximum_iterations = 40;
  int correspondence_randomness = 10;
  double max_correspondence_distance = 0.60;
  double transformation_epsilon = 1e-5;
  double rotation_epsilon = 2e-3;
  double fitness_epsilon = 1e-5;
  double max_fitness_score = 0.08;
  double inlier_distance = 0.30;
  double min_inlier_ratio = 0.35;
  double max_translation = 2.00;
  double max_rotation_rad = 0.2617993877991494;  // 15 degrees.
  // A repetitive tunnel can report an excellent ICP score at a false zero
  // translation. Do not establish the constant-motion predictor from that
  // solution unless explicitly allowed by the operator.
  bool allow_unverified_bootstrap = false;
  bool allow_near_identity_bootstrap = false;
  double minimum_bootstrap_translation = 0.10;
  double minimum_bootstrap_rotation_rad = 0.008726646259971648;  // 0.5 deg.
  double max_prediction_translation_error = 0.75;
  double max_prediction_rotation_error_rad = 0.17453292519943295;  // 10 deg.
  std::size_t min_points = 200;
};

struct RegistrationQuality {
  bool converged = false;
  bool transform_finite = false;
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double translation = 0.0;
  double rotation_rad = 0.0;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
};

struct RegistrationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool accepted = false;
  std::string reason = "not_run";
  Eigen::Matrix4f target_from_source = Eigen::Matrix4f::Identity();
  RegistrationQuality quality;
};

// Evaluates the quality independently from PCL so the fail-closed policy is
// deterministic and unit-testable.
std::string registrationRejectReason(const RegistrationQuality& quality,
                                     const RegistrationConfig& config);

// Checks whether a geometrically accepted relative transform is observable
// enough to start tracking, and later remains consistent with the constant-
// velocity prediction. Returns "accepted" or a structured rejection reason.
std::string motionConsistencyRejectReason(
    const Eigen::Matrix4f& candidate_target_from_source,
    const Eigen::Matrix4f& predicted_target_from_source,
    bool prediction_valid,
    const RegistrationConfig& config,
    double* translation_error = nullptr,
    double* rotation_error_rad = nullptr);

class FrameRegistrar {
 public:
  explicit FrameRegistrar(const RegistrationConfig& config);

  RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
      const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity()) const;

 private:
  RegistrationConfig config_;
};

}  // namespace local_tsdf_mesh
