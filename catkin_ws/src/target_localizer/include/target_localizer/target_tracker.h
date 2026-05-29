#ifndef TARGET_LOCALIZER_TARGET_TRACKER_H
#define TARGET_LOCALIZER_TARGET_TRACKER_H

#include <cstdint>
#include <string>

namespace target_localizer {

enum class TargetStatus : std::uint8_t {
    GOOD = 0,
    DEGRADED = 1,
    LOST = 2,
};

struct TrackerConfig {
    int min_good_inliers = 25;
    double good_residual_rms = 0.015;
    int lost_after_misses = 3;
    double hold_duration = 0.3;
};

struct Measurement {
    double stamp = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int inlier_count = 0;
    double residual_rms = 0.0;
};

struct TrackerOutput {
    double stamp = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    TargetStatus status = TargetStatus::LOST;
};

const char* statusText(TargetStatus status);

class TargetTracker {
public:
    explicit TargetTracker(const TrackerConfig& config);

    TrackerOutput update(const Measurement& measurement);
    TrackerOutput markMissed(double stamp);
    bool hasState() const;

private:
    TrackerConfig config_;
    TrackerOutput state_;
    int consecutive_misses_ = 0;
    bool initialized_ = false;
};

}  // namespace target_localizer

#endif  // TARGET_LOCALIZER_TARGET_TRACKER_H
