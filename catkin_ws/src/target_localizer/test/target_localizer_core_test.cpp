#include <gtest/gtest.h>

#include <Eigen/Core>

#include "target_localizer/cylinder_geometry.h"
#include "target_localizer/equipment_geometry.h"
#include "target_localizer/target_tracker.h"

namespace target_localizer {
namespace {

TEST(CylinderGeometryTest, IntersectsAxisAtReferenceHeight) {
    CylinderModel model;
    model.axis_point = Eigen::Vector3d(-8.0, 0.25, -0.7);
    model.axis_dir = Eigen::Vector3d(0.1, -0.2, 1.0).normalized();

    const Eigen::Vector3d center = centerAtReferenceHeight(model, -0.142);

    EXPECT_NEAR(center.z(), -0.142, 1e-9);
    EXPECT_NEAR(center.x(), -7.9442, 1e-4);
    EXPECT_NEAR(center.y(), 0.1384, 1e-4);
}

TEST(TargetTrackerTest, EmitsGoodThenLostAfterConsecutiveMisses) {
    TrackerConfig config;
    config.min_good_inliers = 25;
    config.good_residual_rms = 0.015;
    config.lost_after_misses = 3;
    config.hold_duration = 0.3;

    TargetTracker tracker(config);
    Measurement measurement;
    measurement.stamp = 10.0;
    measurement.cx = -8.0;
    measurement.cy = 0.0;
    measurement.inlier_count = 42;
    measurement.residual_rms = 0.010;

    const TrackerOutput first = tracker.update(measurement);
    EXPECT_EQ(first.status, TargetStatus::GOOD);
    EXPECT_DOUBLE_EQ(first.cx, -8.0);
    EXPECT_DOUBLE_EQ(first.cy, 0.0);

    EXPECT_EQ(tracker.markMissed(10.1).status, TargetStatus::DEGRADED);
    EXPECT_EQ(tracker.markMissed(10.2).status, TargetStatus::DEGRADED);
    EXPECT_EQ(tracker.markMissed(10.3).status, TargetStatus::LOST);
}

TEST(EquipmentGeometryTest, EstimatesRollPitchFromGroundPlane) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    const double roll = 5.0 * M_PI / 180.0;
    const double pitch = -3.0 * M_PI / 180.0;
    for (double x = -3.0; x <= 3.0; x += 0.3) {
        for (double y = -1.8; y <= 1.8; y += 0.3) {
            pcl::PointXYZI p;
            p.x = x;
            p.y = y;
            p.z = std::tan(pitch) * x - std::tan(roll) * y - 0.8;
            p.intensity = 10.0f;
            cloud.push_back(p);
        }
    }

    EquipmentGeometryConfig config;
    config.min_ground_points = 20;
    config.min_wall_points = 9999;
    const EquipmentGeometryResult result =
        estimateEquipmentGeometry(cloud, config);

    EXPECT_TRUE(result.attitude_valid);
    EXPECT_NEAR(result.roll_deg, 5.0, 0.25);
    EXPECT_NEAR(result.pitch_deg, -3.0, 0.25);
}

TEST(EquipmentGeometryTest, EstimatesYawFromTunnelWalls) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    const double yaw = 8.0 * M_PI / 180.0;
    const double tan_yaw = std::tan(yaw);
    for (double x = -5.0; x <= 5.0; x += 0.25) {
        for (double z = -0.5; z <= 1.5; z += 0.5) {
            pcl::PointXYZI left;
            left.x = x;
            left.y = 2.0 + tan_yaw * x;
            left.z = z;
            cloud.push_back(left);
            pcl::PointXYZI right;
            right.x = x;
            right.y = -2.0 + tan_yaw * x;
            right.z = z;
            cloud.push_back(right);
        }
    }

    EquipmentGeometryConfig config;
    config.min_ground_points = 9999;
    config.min_wall_points = 20;
    const EquipmentGeometryResult result =
        estimateEquipmentGeometry(cloud, config);

    EXPECT_TRUE(result.attitude_valid);
    EXPECT_NEAR(result.yaw_deg, 8.0, 0.25);
}

TEST(EquipmentGeometryTest, MeasuresFourSideDistancesAtConfiguredStations) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto addSide = [&cloud](double x_center, double y) {
        for (double x = x_center - 0.15; x <= x_center + 0.15; x += 0.05) {
            for (double z = -0.5; z <= 0.5; z += 0.25) {
                pcl::PointXYZI p;
                p.x = x;
                p.y = y;
                p.z = z;
                cloud.push_back(p);
            }
        }
    };
    addSide(2.0, 1.30);
    addSide(-1.5, 1.55);
    addSide(2.0, -1.80);
    addSide(-1.5, -2.05);

    EquipmentGeometryConfig config;
    config.front_sample_distance_m = 2.0;
    config.rear_sample_distance_m = 1.5;
    config.sample_window_x_m = 0.4;
    config.min_distance_points = 3;
    const EquipmentGeometryResult result =
        estimateEquipmentGeometry(cloud, config);

    EXPECT_TRUE(result.distances_valid);
    EXPECT_NEAR(result.left_front_mm, 1300.0, 1.0);
    EXPECT_NEAR(result.left_rear_mm, 1550.0, 1.0);
    EXPECT_NEAR(result.right_front_mm, 1800.0, 1.0);
    EXPECT_NEAR(result.right_rear_mm, 2050.0, 1.0);
}

TEST(EquipmentGeometryTest, SubtractsEquipmentHalfWidthFromSideDistances) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto addSide = [&cloud](double x_center, double y) {
        for (double x = x_center - 0.15; x <= x_center + 0.15; x += 0.05) {
            for (double z = -0.5; z <= 0.5; z += 0.25) {
                pcl::PointXYZI p;
                p.x = x;
                p.y = y;
                p.z = z;
                cloud.push_back(p);
            }
        }
    };
    addSide(2.0, 1.70);
    addSide(-1.5, 1.70);
    addSide(2.0, -1.70);
    addSide(-1.5, -1.70);

    EquipmentGeometryConfig config;
    config.front_sample_distance_m = 2.0;
    config.rear_sample_distance_m = 1.5;
    config.sample_window_x_m = 0.4;
    config.min_distance_points = 3;
    config.equipment_half_width_m = 1.2;
    const EquipmentGeometryResult result =
        estimateEquipmentGeometry(cloud, config);

    EXPECT_TRUE(result.distances_valid);
    EXPECT_NEAR(result.left_front_mm, 500.0, 1.0);
    EXPECT_NEAR(result.left_rear_mm, 500.0, 1.0);
    EXPECT_NEAR(result.right_front_mm, 500.0, 1.0);
    EXPECT_NEAR(result.right_rear_mm, 500.0, 1.0);
}

}  // namespace
}  // namespace target_localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
