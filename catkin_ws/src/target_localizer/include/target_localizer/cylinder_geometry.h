#ifndef TARGET_LOCALIZER_CYLINDER_GEOMETRY_H
#define TARGET_LOCALIZER_CYLINDER_GEOMETRY_H

#include <Eigen/Core>

namespace target_localizer {

struct CylinderModel {
    Eigen::Vector3d axis_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_dir = Eigen::Vector3d::UnitZ();
    double radius = 0.125;
    int inlier_count = 0;
    double residual_rms = 0.0;
};

Eigen::Vector3d centerAtReferenceHeight(const CylinderModel& model,
                                        double reference_z);

double radialResidual(const Eigen::Vector3d& point,
                      const CylinderModel& model);

}  // namespace target_localizer

#endif  // TARGET_LOCALIZER_CYLINDER_GEOMETRY_H
