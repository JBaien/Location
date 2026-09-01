#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

using Clock = std::chrono::steady_clock;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Options {
  std::string bag_path;
  std::string topic = "/velodyne_points";
  std::string csv_path;
  int pairs = 100;
  int frame_stride = 1;
  int max_iterations = 40;
  double start_offset_s = 0.0;
  float voxel_m = 0.25F;
  float min_range_m = 0.8F;
  float max_range_m = 50.0F;
  float max_correspondence_m = 1.0F;
  double fitness_threshold_m2 = 0.08;
  double overlap_threshold = 0.55;
  double max_step_m = 1.0;
  double max_rotation_deg = 10.0;
  float guess_x_m = 0.0F;
  bool use_pca_guess = false;
  float guess_pca_m = 0.0F;
};

struct Frame {
  double stamp_s = 0.0;
  std::size_t raw_points = 0;
  Cloud::Ptr points{new Cloud};
};

struct PairResult {
  int pair_index = 0;
  double previous_stamp_s = 0.0;
  double current_stamp_s = 0.0;
  double dt_s = 0.0;
  std::size_t previous_raw_points = 0;
  std::size_t current_raw_points = 0;
  std::size_t previous_voxel_points = 0;
  std::size_t current_voxel_points = 0;
  bool converged = false;
  bool finite_transform = false;
  bool plausible = false;
  double fitness_m2 = std::numeric_limits<double>::infinity();
  double overlap_ratio = 0.0;
  double inlier_rmse_m = std::numeric_limits<double>::infinity();
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double translation_m = 0.0;
  double rotation_deg = 0.0;
  double elapsed_ms = 0.0;
  double guess_x = 0.0;
  double guess_y = 0.0;
  double guess_z = 0.0;
  double pca_axis_x = 0.0;
  double pca_axis_y = 0.0;
  double pca_axis_z = 0.0;
};

void printUsage(const char* program) {
  std::cerr
      << "Usage: " << program << " BAG [options]\n"
      << "  --topic NAME                 PointCloud2 topic (default /velodyne_points)\n"
      << "  --pairs N                   adjacent pairs to align (default 100)\n"
      << "  --start-offset SEC          seconds from bag start\n"
      << "  --frame-stride N            retain every Nth cloud\n"
      << "  --voxel M                   voxel leaf size (default 0.25)\n"
      << "  --min-range M               minimum input range (default 0.8)\n"
      << "  --max-range M               maximum input range (default 50)\n"
      << "  --max-correspondence M      ICP correspondence gate (default 1.0)\n"
      << "  --iterations N              ICP iteration cap (default 40)\n"
      << "  --guess-x M                 fixed source-to-target initial X translation\n"
      << "  --guess-pca M               initial translation along target-cloud principal axis\n"
      << "  --csv PATH                  optional per-pair CSV\n";
}

template <typename T>
T parseNumber(const std::string& value, const std::string& option);

template <>
int parseNumber<int>(const std::string& value, const std::string& option) {
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    throw std::runtime_error("invalid integer for " + option + ": " + value);
  }
}

template <>
float parseNumber<float>(const std::string& value, const std::string& option) {
  try {
    return std::stof(value);
  } catch (const std::exception&) {
    throw std::runtime_error("invalid number for " + option + ": " + value);
  }
}

template <>
double parseNumber<double>(const std::string& value, const std::string& option) {
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    throw std::runtime_error("invalid number for " + option + ": " + value);
  }
}

Options parseOptions(int argc, char** argv) {
  if (argc >= 2 && (std::string(argv[1]) == "--help" ||
                    std::string(argv[1]) == "-h")) {
    printUsage(argv[0]);
    std::exit(0);
  }
  if (argc < 2) {
    printUsage(argv[0]);
    throw std::runtime_error("bag path is required");
  }
  Options options;
  options.bag_path = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value for " + option);
    }
    const std::string value = argv[++index];
    if (option == "--topic") options.topic = value;
    else if (option == "--pairs") options.pairs = parseNumber<int>(value, option);
    else if (option == "--start-offset") options.start_offset_s = parseNumber<double>(value, option);
    else if (option == "--frame-stride") options.frame_stride = parseNumber<int>(value, option);
    else if (option == "--voxel") options.voxel_m = parseNumber<float>(value, option);
    else if (option == "--min-range") options.min_range_m = parseNumber<float>(value, option);
    else if (option == "--max-range") options.max_range_m = parseNumber<float>(value, option);
    else if (option == "--max-correspondence") options.max_correspondence_m = parseNumber<float>(value, option);
    else if (option == "--iterations") options.max_iterations = parseNumber<int>(value, option);
    else if (option == "--guess-x") options.guess_x_m = parseNumber<float>(value, option);
    else if (option == "--guess-pca") {
      options.use_pca_guess = true;
      options.guess_pca_m = parseNumber<float>(value, option);
    }
    else if (option == "--csv") options.csv_path = value;
    else throw std::runtime_error("unknown option: " + option);
  }
  if (options.pairs <= 0 || options.frame_stride <= 0 || options.max_iterations <= 0) {
    throw std::runtime_error("pairs, frame-stride, and iterations must be positive");
  }
  if (options.voxel_m <= 0.0F || options.max_correspondence_m <= 0.0F ||
      options.max_range_m <= options.min_range_m) {
    throw std::runtime_error("invalid range, voxel, or correspondence option");
  }
  return options;
}

Cloud::Ptr filterCloud(const sensor_msgs::PointCloud2& message, const Options& options) {
  Cloud::Ptr decoded(new Cloud);
  pcl::fromROSMsg(message, *decoded);
  Cloud::Ptr cropped(new Cloud);
  cropped->reserve(decoded->size());
  const float minimum_squared = options.min_range_m * options.min_range_m;
  const float maximum_squared = options.max_range_m * options.max_range_m;
  for (const auto& point : decoded->points) {
    if (!pcl::isFinite(point)) continue;
    const float squared_range = point.x * point.x + point.y * point.y + point.z * point.z;
    if (squared_range < minimum_squared || squared_range > maximum_squared) continue;
    cropped->push_back(point);
  }
  Cloud::Ptr downsampled(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(options.voxel_m, options.voxel_m, options.voxel_m);
  voxel.setInputCloud(cropped);
  voxel.filter(*downsampled);
  return downsampled;
}

std::vector<Frame> loadFrames(const Options& options) {
  rosbag::Bag bag;
  bag.open(options.bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery({options.topic}));
  if (view.size() == 0U) {
    bag.close();
    throw std::runtime_error("topic has no messages: " + options.topic);
  }
  const double start_s = view.getBeginTime().toSec() + std::max(0.0, options.start_offset_s);
  const std::size_t required = static_cast<std::size_t>(options.pairs + 1);
  std::vector<Frame> frames;
  frames.reserve(required);
  int eligible_index = 0;
  BOOST_FOREACH (const rosbag::MessageInstance& instance, view) {
    if (instance.getTime().toSec() < start_s) continue;
    auto message = instance.instantiate<sensor_msgs::PointCloud2>();
    if (!message) continue;
    if ((eligible_index++ % options.frame_stride) != 0) continue;
    Frame frame;
    frame.stamp_s = message->header.stamp.isZero()
                        ? instance.getTime().toSec()
                        : message->header.stamp.toSec();
    frame.raw_points = static_cast<std::size_t>(message->width) * message->height;
    frame.points = filterCloud(*message, options);
    frames.push_back(std::move(frame));
    if (frames.size() >= required) break;
  }
  bag.close();
  if (frames.size() < 2U) throw std::runtime_error("fewer than two usable frames were loaded");
  return frames;
}

double rotationAngleDegrees(const Eigen::Matrix3f& rotation) {
  const double cosine = std::max(-1.0, std::min(1.0, (static_cast<double>(rotation.trace()) - 1.0) * 0.5));
  return std::acos(cosine) * 180.0 / M_PI;
}

std::pair<double, double> overlapMetrics(
    const Cloud::ConstPtr& aligned,
    const Cloud::ConstPtr& target,
    float maximum_distance) {
  if (aligned->empty() || target->empty()) return {0.0, std::numeric_limits<double>::infinity()};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  std::vector<int> index(1);
  std::vector<float> squared_distance(1);
  const float maximum_squared = maximum_distance * maximum_distance;
  std::size_t inliers = 0;
  double squared_sum = 0.0;
  for (const auto& point : aligned->points) {
    if (tree.nearestKSearch(point, 1, index, squared_distance) <= 0) continue;
    if (squared_distance[0] > maximum_squared) continue;
    ++inliers;
    squared_sum += squared_distance[0];
  }
  const double overlap = static_cast<double>(inliers) / aligned->size();
  const double rmse = inliers > 0U ? std::sqrt(squared_sum / inliers)
                                   : std::numeric_limits<double>::infinity();
  return {overlap, rmse};
}

PairResult alignPair(const Frame& previous, const Frame& current, int pair_index, const Options& options) {
  PairResult result;
  result.pair_index = pair_index;
  result.previous_stamp_s = previous.stamp_s;
  result.current_stamp_s = current.stamp_s;
  result.dt_s = current.stamp_s - previous.stamp_s;
  result.previous_raw_points = previous.raw_points;
  result.current_raw_points = current.raw_points;
  result.previous_voxel_points = previous.points->size();
  result.current_voxel_points = current.points->size();
  if (previous.points->size() < 50U || current.points->size() < 50U) return result;

  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  pcl::compute3DCentroid(*previous.points, centroid);
  pcl::computeCovarianceMatrixNormalized(*previous.points, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  Eigen::Vector3f principal_axis = Eigen::Vector3f::UnitX();
  if (eigen_solver.info() == Eigen::Success) {
    principal_axis = eigen_solver.eigenvectors().col(2).normalized();
    if (principal_axis.x() < 0.0F) principal_axis = -principal_axis;
  }
  result.pca_axis_x = principal_axis.x();
  result.pca_axis_y = principal_axis.y();
  result.pca_axis_z = principal_axis.z();
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  Eigen::Vector3f guess(options.guess_x_m, 0.0F, 0.0F);
  if (options.use_pca_guess) guess += options.guess_pca_m * principal_axis;
  initial_guess.block<3, 1>(0, 3) = guess;
  result.guess_x = guess.x();
  result.guess_y = guess.y();
  result.guess_z = guess.z();

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(current.points);
  icp.setInputTarget(previous.points);
  icp.setMaximumIterations(options.max_iterations);
  icp.setMaxCorrespondenceDistance(options.max_correspondence_m);
  icp.setTransformationEpsilon(1e-7);
  icp.setEuclideanFitnessEpsilon(1e-7);
  icp.setRANSACOutlierRejectionThreshold(options.max_correspondence_m * 0.5F);
  Cloud::Ptr aligned(new Cloud);
  const auto started = Clock::now();
  icp.align(*aligned, initial_guess);
  const auto finished = Clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(finished - started).count();
  result.converged = icp.hasConverged();
  result.fitness_m2 = icp.getFitnessScore(options.max_correspondence_m);
  const Eigen::Matrix4f transform = icp.getFinalTransformation();
  result.finite_transform = transform.allFinite();
  if (!result.finite_transform) return result;
  result.tx = transform(0, 3);
  result.ty = transform(1, 3);
  result.tz = transform(2, 3);
  result.translation_m = transform.block<3, 1>(0, 3).norm();
  result.rotation_deg = rotationAngleDegrees(transform.block<3, 3>(0, 0));
  const auto overlap = overlapMetrics(aligned, previous.points, options.max_correspondence_m);
  result.overlap_ratio = overlap.first;
  result.inlier_rmse_m = overlap.second;
  result.plausible = result.converged && std::isfinite(result.fitness_m2) &&
                     result.fitness_m2 <= options.fitness_threshold_m2 &&
                     result.overlap_ratio >= options.overlap_threshold &&
                     result.translation_m <= options.max_step_m &&
                     result.rotation_deg <= options.max_rotation_deg && result.dt_s > 0.0;
  return result;
}

double quantile(std::vector<double> values, double fraction) {
  values.erase(std::remove_if(values.begin(), values.end(), [](double value) { return !std::isfinite(value); }), values.end());
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - lower;
  return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

void writeCsv(const std::string& path, const std::vector<PairResult>& results) {
  if (path.empty()) return;
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot open CSV output: " + path);
  stream << "pair,previous_stamp_s,current_stamp_s,dt_s,previous_raw_points,current_raw_points,"
            "previous_voxel_points,current_voxel_points,converged,finite_transform,plausible,"
            "fitness_m2,overlap_ratio,inlier_rmse_m,guess_x,guess_y,guess_z,pca_axis_x,pca_axis_y,"
            "pca_axis_z,tx,ty,tz,translation_m,rotation_deg,elapsed_ms\n";
  stream << std::setprecision(12);
  for (const auto& result : results) {
    stream << result.pair_index << ',' << result.previous_stamp_s << ',' << result.current_stamp_s << ','
           << result.dt_s << ',' << result.previous_raw_points << ',' << result.current_raw_points << ','
           << result.previous_voxel_points << ',' << result.current_voxel_points << ','
           << (result.converged ? 1 : 0) << ',' << (result.finite_transform ? 1 : 0) << ','
           << (result.plausible ? 1 : 0) << ',' << result.fitness_m2 << ',' << result.overlap_ratio << ','
           << result.inlier_rmse_m << ',' << result.guess_x << ',' << result.guess_y << ','
           << result.guess_z << ',' << result.pca_axis_x << ',' << result.pca_axis_y << ','
           << result.pca_axis_z << ',' << result.tx << ',' << result.ty << ',' << result.tz << ','
           << result.translation_m << ',' << result.rotation_deg << ',' << result.elapsed_ms << '\n';
  }
}

void printSummary(const Options& options, const std::vector<Frame>& frames, const std::vector<PairResult>& results) {
  std::vector<double> dt;
  std::vector<double> fitness;
  std::vector<double> overlap;
  std::vector<double> translation;
  std::vector<double> rotation;
  std::vector<double> elapsed;
  std::vector<double> voxel_points;
  std::size_t converged = 0;
  std::size_t plausible = 0;
  for (const auto& frame : frames) voxel_points.push_back(frame.points->size());
  for (const auto& result : results) {
    dt.push_back(result.dt_s);
    fitness.push_back(result.fitness_m2);
    overlap.push_back(result.overlap_ratio);
    translation.push_back(result.translation_m);
    rotation.push_back(result.rotation_deg);
    elapsed.push_back(result.elapsed_ms);
    if (result.converged) ++converged;
    if (result.plausible) ++plausible;
  }
  const double count = static_cast<double>(results.size());
  std::cout << std::setprecision(9)
            << "{\n"
            << "  \"bag\": \"" << options.bag_path << "\",\n"
            << "  \"topic\": \"" << options.topic << "\",\n"
            << "  \"pairs\": " << results.size() << ",\n"
            << "  \"frame_stride\": " << options.frame_stride << ",\n"
            << "  \"voxel_m\": " << options.voxel_m << ",\n"
            << "  \"max_correspondence_m\": " << options.max_correspondence_m << ",\n"
            << "  \"guess_x_m\": " << options.guess_x_m << ",\n"
            << "  \"guess_pca_m\": " << (options.use_pca_guess ? options.guess_pca_m : 0.0F) << ",\n"
            << "  \"converged_ratio\": " << (count > 0.0 ? converged / count : 0.0) << ",\n"
            << "  \"plausible_ratio\": " << (count > 0.0 ? plausible / count : 0.0) << ",\n"
            << "  \"dt_s_median\": " << quantile(dt, 0.5) << ",\n"
            << "  \"voxel_points_median\": " << quantile(voxel_points, 0.5) << ",\n"
            << "  \"fitness_m2_median\": " << quantile(fitness, 0.5) << ",\n"
            << "  \"fitness_m2_p95\": " << quantile(fitness, 0.95) << ",\n"
            << "  \"overlap_median\": " << quantile(overlap, 0.5) << ",\n"
            << "  \"overlap_p10\": " << quantile(overlap, 0.1) << ",\n"
            << "  \"translation_m_median\": " << quantile(translation, 0.5) << ",\n"
            << "  \"translation_m_p95\": " << quantile(translation, 0.95) << ",\n"
            << "  \"rotation_deg_median\": " << quantile(rotation, 0.5) << ",\n"
            << "  \"rotation_deg_p95\": " << quantile(rotation, 0.95) << ",\n"
            << "  \"icp_ms_median\": " << quantile(elapsed, 0.5) << ",\n"
            << "  \"icp_ms_p95\": " << quantile(elapsed, 0.95) << ",\n"
            << "  \"warning\": \"No odometry ground truth is present; convergence is feasibility evidence, not accuracy.\"\n"
            << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const auto frames = loadFrames(options);
    std::vector<PairResult> results;
    results.reserve(frames.size() - 1U);
    for (std::size_t index = 1; index < frames.size(); ++index) {
      results.push_back(alignPair(frames[index - 1U], frames[index], static_cast<int>(index - 1U), options));
    }
    writeCsv(options.csv_path, results);
    printSummary(options, frames, results);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "tunnel_icp_baseline: " << error.what() << '\n';
    return 2;
  }
}
