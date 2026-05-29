#include "target_localizer/cylinder_geometry.h"

#include <cmath>
#include <stdexcept>

namespace target_localizer {

Eigen::Vector3d centerAtReferenceHeight(const CylinderModel& model,
                                        double reference_z) {
    const Eigen::Vector3d axis = model.axis_dir.normalized();
    if (std::abs(axis.z()) < 1e-9) {
        throw std::runtime_error("Cylinder axis is parallel to reference plane");
    }
    const double t = (reference_z - model.axis_point.z()) / axis.z();
    return model.axis_point + t * axis;
}

double radialResidual(const Eigen::Vector3d& point,
                      const CylinderModel& model) {
    const Eigen::Vector3d axis = model.axis_dir.normalized();
    const Eigen::Vector3d delta = point - model.axis_point;
    const Eigen::Vector3d radial = delta - delta.dot(axis) * axis;
    return radial.norm() - model.radius;
}

}  // namespace target_localizer
