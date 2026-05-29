#include <gtest/gtest.h>

#include <Eigen/Core>

#include "target_localizer/cylinder_geometry.h"
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

}  // namespace
}  // namespace target_localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
