#include "target_localizer/equipment_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
        sample.mm = percentile(distances, config.distance_percentile) * 1000.0;
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
    std::vector<Eigen::Vector2d> wall_points;
    ground_points.reserve(cloud.size());
    wall_points.reserve(cloud.size());

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
        // 偏航角来自巷道左右边界的主方向。这里只用 |y| 较大的侧壁/边界点，
        // 并投影到 XY 平面拟合 y = kx + b，k 对应设备前向和巷道方向的夹角。
        if (inRange(point.x, config.wall_x_min, config.wall_x_max) &&
            std::abs(point.y) >= config.wall_y_abs_min &&
            std::abs(point.y) <= config.wall_y_abs_max &&
            inRange(point.z, config.wall_z_min, config.wall_z_max)) {
            wall_points.emplace_back(point.x, point.y);
        }
    }

    result.ground_points = static_cast<int>(ground_points.size());
    result.wall_points = static_cast<int>(wall_points.size());

    bool plane_valid = false;
    if (result.ground_points >= config.min_ground_points) {
        Eigen::MatrixXd a(result.ground_points, 3);
        Eigen::VectorXd b(result.ground_points);
        for (int i = 0; i < result.ground_points; ++i) {
            a(i, 0) = ground_points[i].x();
            a(i, 1) = ground_points[i].y();
            a(i, 2) = 1.0;
            b(i) = ground_points[i].z();
        }
        // 以 z = ax + by + c 拟合局部地面。a 反映前后坡度，b 反映左右坡度。
        // pitch = atan(a)，roll = atan(-b)：符号约定与 base_link 中 X 前、Y 左、
        // Z 上的常见车辆坐标系一致，便于和惯导 roll/pitch 做同屏对比。
        const Eigen::Vector3d coeff =
            a.colPivHouseholderQr().solve(b);
        result.pitch_deg = std::atan(coeff.x()) * kRadToDeg;
        result.roll_deg = std::atan(-coeff.y()) * kRadToDeg;
        plane_valid = std::isfinite(result.pitch_deg) &&
                      std::isfinite(result.roll_deg);
    }

    bool yaw_valid = false;
    if (result.wall_points >= config.min_wall_points) {
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto& point : wall_points) {
            mean_x += point.x();
            mean_y += point.y();
        }
        mean_x /= static_cast<double>(wall_points.size());
        mean_y /= static_cast<double>(wall_points.size());

        double sxx = 0.0;
        double sxy = 0.0;
        for (const auto& point : wall_points) {
            const double dx = point.x() - mean_x;
            sxx += dx * dx;
            sxy += dx * (point.y() - mean_y);
        }
        if (std::abs(sxx) > 1e-9) {
            // 两侧墙点一起参与拟合时，截距会互相抵消，但主方向斜率会保留；
            // 因此这里不区分左右墙，只估计整体巷道走向。点数不足时保持 yaw=0
            // 并通过 quality=degraded/lost 告知上层不要强信任该角度。
            const double slope = sxy / sxx;
            result.yaw_deg = std::atan(slope) * kRadToDeg;
            yaw_valid = std::isfinite(result.yaw_deg);
        }
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
