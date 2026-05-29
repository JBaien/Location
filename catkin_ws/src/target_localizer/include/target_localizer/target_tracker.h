#ifndef TARGET_LOCALIZER_TARGET_TRACKER_H
#define TARGET_LOCALIZER_TARGET_TRACKER_H

#include <cstdint>
#include <string>

namespace target_localizer {

// 输出质量状态：
// GOOD：内点数和残差均满足阈值；
// DEGRADED：仍有输出，但质量不足或短时丢失后处于保活；
// LOST：连续多帧无有效测量，不再认为当前输出可靠。
enum class TargetStatus : std::uint8_t {
    GOOD = 0,
    DEGRADED = 1,
    LOST = 2,
};

// 跟踪器只做轻量级常速度估计和状态机，不承担点云检测职责。
// 阈值来自 target_localizer.yaml，便于现场按点数、粉尘、遮挡情况调整。
struct TrackerConfig {
    int min_good_inliers = 25;
    double good_residual_rms = 0.015;
    int lost_after_misses = 3;
    double hold_duration = 0.3;
};

// 单帧圆柱检测结果。这里的 cx/cy 是目标圆柱轴线在参考高度处的中心点。
struct Measurement {
    double stamp = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int inlier_count = 0;
    double residual_rms = 0.0;
};

// 滤波后的业务输出。dx/dy 不放在这里计算，而是在 ROS 发布层按建零点统一换算。
struct TrackerOutput {
    double stamp = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    TargetStatus status = TargetStatus::LOST;
};

const char* statusText(TargetStatus status);

// 简化版常速度跟踪器：
// 1. 有新测量时，用相邻两帧中心点差分估计速度；
// 2. 短时无测量时，按上一速度保活；
// 3. 连续丢失达到阈值后进入 LOST。
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
