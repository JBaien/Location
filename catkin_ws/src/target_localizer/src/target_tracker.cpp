#include "target_localizer/target_tracker.h"

#include <algorithm>

namespace target_localizer {

const char* statusText(TargetStatus status) {
    switch (status) {
        case TargetStatus::GOOD:
            return "GOOD";
        case TargetStatus::DEGRADED:
            return "DEGRADED";
        case TargetStatus::LOST:
            return "LOST";
    }
    return "LOST";
}

TargetTracker::TargetTracker(const TrackerConfig& config) : config_(config) {}

TrackerOutput TargetTracker::update(const Measurement& measurement) {
    TrackerOutput next;
    next.stamp = measurement.stamp;
    next.cx = measurement.cx;
    next.cy = measurement.cy;

    if (initialized_) {
        const double dt = std::max(1e-3, measurement.stamp - state_.stamp);
        next.vx = (measurement.cx - state_.cx) / dt;
        next.vy = (measurement.cy - state_.cy) / dt;
    }

    const bool good = measurement.inlier_count >= config_.min_good_inliers &&
                      measurement.residual_rms <= config_.good_residual_rms;
    next.status = good ? TargetStatus::GOOD : TargetStatus::DEGRADED;

    state_ = next;
    initialized_ = true;
    consecutive_misses_ = 0;
    return state_;
}

TrackerOutput TargetTracker::markMissed(double stamp) {
    if (!initialized_) {
        TrackerOutput lost;
        lost.stamp = stamp;
        lost.status = TargetStatus::LOST;
        return lost;
    }

    ++consecutive_misses_;
    const double dt = std::max(0.0, stamp - state_.stamp);
    state_.stamp = stamp;
    if (dt <= config_.hold_duration) {
        state_.cx += state_.vx * dt;
        state_.cy += state_.vy * dt;
    }
    state_.status = consecutive_misses_ >= config_.lost_after_misses
                        ? TargetStatus::LOST
                        : TargetStatus::DEGRADED;
    return state_;
}

bool TargetTracker::hasState() const { return initialized_; }

}  // namespace target_localizer
