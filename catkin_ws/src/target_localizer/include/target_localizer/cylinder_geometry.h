#ifndef TARGET_LOCALIZER_CYLINDER_GEOMETRY_H
#define TARGET_LOCALIZER_CYLINDER_GEOMETRY_H

#include <Eigen/Core>

namespace target_localizer {

// 圆柱模型采用 PCL RANSAC 输出的标准形式：
//   axis_point：轴线上的一点
//   axis_dir：轴线方向向量
//   radius：圆柱半径
// 定位业务最终只关心参考高度处的轴线中心点，而不是完整三维姿态。
struct CylinderModel {
    Eigen::Vector3d axis_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_dir = Eigen::Vector3d::UnitZ();
    double radius = 0.125;
    int inlier_count = 0;
    double residual_rms = 0.0;
};

// 计算圆柱轴线与 z=reference_z 平面的交点。
// 这个点作为当前帧的目标中心，后续与建零点 (zero_x, zero_y) 做差得到 XY 位移。
Eigen::Vector3d centerAtReferenceHeight(const CylinderModel& model,
                                        double reference_z);

// 计算点到圆柱侧壁的径向残差：
//   residual = 点到轴线距离 - 圆柱半径
// 用于估计拟合 RMS，进而驱动 GOOD / DEGRADED / LOST 状态判断。
double radialResidual(const Eigen::Vector3d& point,
                      const CylinderModel& model);

}  // namespace target_localizer

#endif  // TARGET_LOCALIZER_CYLINDER_GEOMETRY_H
