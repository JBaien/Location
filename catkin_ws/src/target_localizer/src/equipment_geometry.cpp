#include "target_localizer/equipment_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Dense>

namespace target_localizer {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

bool finitePoint(const pcl::PointXYZI& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bool inRange(double value, double min_value, double max_value) {
    return value >= min_value && value <= max_value;
}

double percentile(std::vector<double> values, double ratio) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    ratio = std::max(0.0, std::min(1.0, ratio));
    const std::size_t index = static_cast<std::size_t>(
        std::floor(ratio * static_cast<double>(values.size() - 1U)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

struct DistanceSample {
    double mm = 0.0;
    int points = 0;
    bool valid = false;
};

struct PlaneFit {
    bool valid = false;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double rmse = 0.0;
    double normal_z = 0.0;
    int inliers = 0;
};

bool fitPcaDirection(const std::vector<Eigen::Vector2d>& points,
                     Eigen::Vector2d& direction) {
    if (points.size() < 2U) {
        return false;
    }
    Eigen::Vector2d mean(0.0, 0.0);
    for (const auto& point : points) {
        mean += point;
    }
    mean /= static_cast<double>(points.size());

    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    for (const auto& point : points) {
        const double dx = point.x() - mean.x();
        const double dy = point.y() - mean.y();
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    direction = Eigen::Vector2d(std::cos(theta), std::sin(theta));
    if (direction.x() < 0.0) {
        direction = -direction;
    }
    return direction.allFinite() && direction.norm() > 1e-6;
}

bool solvePlaneLeastSquares(const std::vector<Eigen::Vector3d>& points,
                            PlaneFit& fit) {
    if (points.size() < 3U) {
        return false;
    }
    Eigen::MatrixXd a(points.size(), 3);
    Eigen::VectorXd b(points.size());
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        a(i, 0) = points[i].x();
        a(i, 1) = points[i].y();
        a(i, 2) = 1.0;
        b(i) = points[i].z();
    }
    const Eigen::Vector3d coeff = a.colPivHouseholderQr().solve(b);
    if (!coeff.allFinite()) {
        return false;
    }

    double sq_error = 0.0;
    for (const auto& point : points) {
        const double residual =
            point.z() - (coeff.x() * point.x() + coeff.y() * point.y() + coeff.z());
        sq_error += residual * residual;
    }
    const Eigen::Vector3d normal(-coeff.x(), -coeff.y(), 1.0);
    fit.a = coeff.x();
    fit.b = coeff.y();
    fit.c = coeff.z();
    fit.rmse = std::sqrt(sq_error / static_cast<double>(points.size()));
    fit.normal_z = std::abs(normal.normalized().z());
    fit.inliers = static_cast<int>(points.size());
    fit.valid = std::isfinite(fit.rmse) && std::isfinite(fit.normal_z);
    return fit.valid;
}

PlaneFit fitGroundPlaneRansac(const std::vector<Eigen::Vector3d>& points,
                              const EquipmentGeometryConfig& config) {
    PlaneFit best;
    if (points.size() < static_cast<std::size_t>(config.min_ground_points)) {
        return best;
    }

    constexpr int kIterations = 80;
    const double threshold = std::max(0.02, config.max_ground_plane_rmse);
    std::mt19937 rng(7);
    std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1U);

    std::vector<Eigen::Vector3d> best_inliers;
    for (int i = 0; i < kIterations; ++i) {
        const auto& p1 = points[pick(rng)];
        const auto& p2 = points[pick(rng)];
        const auto& p3 = points[pick(rng)];
        const Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1);
        if (normal.norm() < 1e-6 || std::abs(normal.z()) < 1e-6) {
            continue;
        }
        const double a = -normal.x() / normal.z();
        const double b = -normal.y() / normal.z();
        const double c = normal.dot(p1) / normal.z();
        std::vector<Eigen::Vector3d> inliers;
        inliers.reserve(points.size());
        for (const auto& point : points) {
            const double residual = std::abs(point.z() - (a * point.x() + b * point.y() + c));
            if (residual <= threshold) {
                inliers.push_back(point);
            }
        }
        if (inliers.size() > best_inliers.size()) {
            best_inliers = std::move(inliers);
        }
    }

    if (best_inliers.size() < static_cast<std::size_t>(config.min_ground_points)) {
        return best;
    }
    solvePlaneLeastSquares(best_inliers, best);
    best.valid = best.valid &&
                 best.rmse <= config.max_ground_plane_rmse &&
                 best.normal_z >= config.min_ground_normal_z;
    return best;
}

DistanceSample sampleSideDistance(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                  double station_x,
                                  int side_sign,
                                  const EquipmentGeometryConfig& config) {
    std::vector<double> distances;
    const double half_window = std::max(0.01, config.sample_window_x_m * 0.5);
    for (const auto& point : cloud.points) {
        if (!finitePoint(point)) {
            continue;
        }
        if (std::abs(point.x - station_x) > half_window ||
            !inRange(point.z, config.side_z_min, config.side_z_max)) {
            continue;
        }
        const double signed_y = static_cast<double>(side_sign) * point.y;
        if (signed_y < config.side_y_abs_min ||
            signed_y > config.side_y_abs_max) {
            continue;
        }
        // 四角距离不直接取最小值，避免粉尘点、反射孤点或边界毛刺把距离拉得过近。
        // 默认取靠近设备一侧的 10% 分位数：比均值更贴近最近有效边界，比 min 稳定。
        distances.push_back(signed_y);
    }

    DistanceSample sample;
    sample.points = static_cast<int>(distances.size());
    if (sample.points >= config.min_distance_points) {
        const double wall_distance =
            percentile(distances, config.distance_percentile);
        const double clearance =
            wall_distance - std::max(0.0, config.equipment_half_width_m);
        sample.mm = std::max(0.0, clearance) * 1000.0;
        sample.valid = std::isfinite(sample.mm);
    }
    return sample;
}

}  // namespace

EquipmentGeometryResult estimateEquipmentGeometry(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const EquipmentGeometryConfig& config) {
    EquipmentGeometryResult result;

    std::vector<Eigen::Vector3d> ground_points;
    std::vector<Eigen::Vector2d> left_wall_points;
    std::vector<Eigen::Vector2d> right_wall_points;
    ground_points.reserve(cloud.size());
    left_wall_points.reserve(cloud.size() / 2U);
    right_wall_points.reserve(cloud.size() / 2U);

    for (const auto& point : cloud.points) {
        if (!finitePoint(point)) {
            continue;
        }
        // 地面 ROI 只保留设备附近的底板/路面点。姿态计算依赖局部地面，
        // 不应把远处墙面、支架、车体结构混进来，否则 roll/pitch 会被拉偏。
        if (inRange(point.x, config.ground_x_min, config.ground_x_max) &&
            inRange(point.y, config.ground_y_min, config.ground_y_max) &&
            inRange(point.z, config.ground_z_min, config.ground_z_max)) {
            ground_points.emplace_back(point.x, point.y, point.z);
        }
        // 偏航角来自巷道左右边界的主方向。左右墙分开做 PCA，避免两堵墙的
        // 横向间距把主方向拉偏；再把方向统一到 +X 半平面后求平均。
        if (inRange(point.x, config.wall_x_min, config.wall_x_max) &&
            std::abs(point.y) >= config.wall_y_abs_min &&
            std::abs(point.y) <= config.wall_y_abs_max &&
            inRange(point.z, config.wall_z_min, config.wall_z_max)) {
            if (point.y >= 0.0) {
                left_wall_points.emplace_back(point.x, point.y);
            } else {
                right_wall_points.emplace_back(point.x, point.y);
            }
        }
    }

    result.ground_points = static_cast<int>(ground_points.size());
    result.wall_points = static_cast<int>(left_wall_points.size() +
                                          right_wall_points.size());

    bool plane_valid = false;
    const PlaneFit ground = fitGroundPlaneRansac(ground_points, config);
    if (ground.valid) {
        // RANSAC 先剔除浮煤、车体结构、线缆等离群点，再用内点最小二乘细化平面。
        // 通过法向量 n=[-a,-b,1] 和 normal_z/RMSE 做有效性约束，避免墙面或障碍物
        // 被误当作底板。符号约定对应 base_link 中 X 前、Y 左、Z 上。
        result.pitch_deg = std::atan2(ground.a, 1.0) * kRadToDeg;
        result.roll_deg = std::atan2(-ground.b, 1.0) * kRadToDeg;
        result.ground_plane_rmse = ground.rmse;
        result.ground_points = ground.inliers;
        plane_valid = std::isfinite(result.pitch_deg) &&
                      std::isfinite(result.roll_deg);
        result.pitch_valid = plane_valid;
        result.roll_valid = plane_valid;
    }

    bool yaw_valid = false;
    Eigen::Vector2d direction_sum(0.0, 0.0);
    int direction_count = 0;
    Eigen::Vector2d direction;
    if (static_cast<int>(left_wall_points.size()) >= config.min_wall_points &&
        fitPcaDirection(left_wall_points, direction)) {
        direction_sum += direction;
        ++direction_count;
    }
    if (static_cast<int>(right_wall_points.size()) >= config.min_wall_points &&
        fitPcaDirection(right_wall_points, direction)) {
        direction_sum += direction;
        ++direction_count;
    }
    if (direction_count > 0 && direction_sum.norm() > 1e-6) {
        const Eigen::Vector2d dir = direction_sum.normalized();
        result.yaw_deg = std::atan2(dir.y(), dir.x()) * kRadToDeg;
        yaw_valid = std::isfinite(result.yaw_deg);
        result.yaw_valid = yaw_valid;
    }

    const double front_x =
        static_cast<double>(config.forward_sign) * config.front_sample_distance_m;
    const double rear_x =
        -static_cast<double>(config.forward_sign) * config.rear_sample_distance_m;
    const int left_side = config.left_sign >= 0 ? 1 : -1;
    const int right_side = -left_side;

    const DistanceSample lf = sampleSideDistance(cloud, front_x, left_side, config);
    const DistanceSample lr = sampleSideDistance(cloud, rear_x, left_side, config);
    const DistanceSample rf = sampleSideDistance(cloud, front_x, right_side, config);
    const DistanceSample rr = sampleSideDistance(cloud, rear_x, right_side, config);
    result.left_front_mm = lf.mm;
    result.left_rear_mm = lr.mm;
    result.right_front_mm = rf.mm;
    result.right_rear_mm = rr.mm;
    result.left_front_points = lf.points;
    result.left_rear_points = lr.points;
    result.right_front_points = rf.points;
    result.right_rear_points = rr.points;
    result.left_front_valid = lf.valid;
    result.left_rear_valid = lr.valid;
    result.right_front_valid = rf.valid;
    result.right_rear_valid = rr.valid;
    result.distances_valid = lf.valid && lr.valid && rf.valid && rr.valid;
    result.attitude_valid = plane_valid || yaw_valid;
    if (plane_valid && yaw_valid && result.distances_valid) {
        result.quality = "good";
    } else if (result.attitude_valid || result.distances_valid) {
        result.quality = "degraded";
    } else {
        result.quality = "lost";
    }
    return result;
}

}  // namespace target_localizer
