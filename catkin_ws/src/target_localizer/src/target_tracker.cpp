#include "target_localizer/target_tracker.h"

#include <algorithm>
#include <cmath>

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

bool TargetTracker::acceptsMeasurement(const Measurement& measurement) const {
    if (measurement.inlier_count < config_.min_update_inliers ||
        !std::isfinite(measurement.residual_rms) ||
        measurement.residual_rms > config_.max_update_residual_rms ||
        !std::isfinite(measurement.cx) || !std::isfinite(measurement.cy)) {
        return false;
    }

    if (!initialized_) {
        return true;
    }

    const double dx = measurement.cx - state_.cx;
    const double dy = measurement.cy - state_.cy;
    const double jump = std::hypot(dx, dy);
    if (config_.max_update_jump_m > 0.0 && jump > config_.max_update_jump_m) {
        return false;
    }

    const double dt = measurement.stamp - last_measurement_stamp_;
    if (config_.max_velocity_mps > 0.0 && dt > 1e-3) {
        const double velocity = jump / dt;
        if (velocity > config_.max_velocity_mps) {
            return false;
        }
    }
    return true;
}

TrackerOutput TargetTracker::update(const Measurement& measurement) {
    if (!acceptsMeasurement(measurement)) {
        return markMissed(measurement.stamp);
    }

    TrackerOutput next;
    next.stamp = measurement.stamp;
    next.cx = measurement.cx;
    next.cy = measurement.cy;

    if (initialized_) {
        const double dt =
            std::max(1e-3, measurement.stamp - last_measurement_stamp_);
        next.vx = (measurement.cx - state_.cx) / dt;
        next.vy = (measurement.cy - state_.cy) / dt;
    }

    const bool good = measurement.inlier_count >= config_.min_good_inliers &&
                      measurement.residual_rms <= config_.good_residual_rms;
    next.status = good ? TargetStatus::GOOD : TargetStatus::DEGRADED;

    state_ = next;
    initialized_ = true;
    last_measurement_stamp_ = measurement.stamp;
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
    const bool lost = consecutive_misses_ >= config_.lost_after_misses;
    const double dt_since_measurement =
        std::max(0.0, stamp - last_measurement_stamp_);
    if (!lost && config_.hold_duration > 0.0 &&
        dt_since_measurement <= config_.hold_duration) {
        const double dt = std::max(0.0, stamp - state_.stamp);
        state_.cx += state_.vx * dt;
        state_.cy += state_.vy * dt;
    }
    state_.stamp = stamp;
    state_.status = lost ? TargetStatus::LOST : TargetStatus::DEGRADED;
    if (lost) {
        state_.vx = 0.0;
        state_.vy = 0.0;
    }
    return state_;
}

bool TargetTracker::hasState() const { return initialized_; }

}  // namespace target_localizer
