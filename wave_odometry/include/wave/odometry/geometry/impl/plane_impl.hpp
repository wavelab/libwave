#ifndef WAVE_PLANE_IMPL_HPP
#define WAVE_PLANE_IMPL_HPP

#include "wave/odometry/geometry/plane.hpp"

namespace wave {

template <int... states>
bool PlaneResidual<states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> error(residuals);
    Eigen::Map<const Vec6> plane(&parameters[0][0]);

    // Calculate error
    Eigen::Map<const Vec3f> Pt(this->pt);
    Vec3 diff = Pt.cast<double>() - plane.block<3, 1>(3, 0);
    error = diff.transpose() * plane.block<3, 1>(0, 0);

    if (std::isnan(residuals[0])) {
        throw std::runtime_error("nan in plane residual, probably due to incorrect point index");
    }

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> plane_jac(jacobians[0]);

            plane_jac.template block<1, 3>(0, 0) = diff.transpose();
            plane_jac.template block<1, 3>(0, 3) = -plane.block<3, 1>(0, 0).transpose();
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        del_ptT_T << 0, Pt(2), -Pt(1), 1, 0, 0, -Pt(2), 0, Pt(0), 0, 1, 0, Pt(1), -Pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 1, 6> del_e_del_T;
        del_e_del_T = plane.block<3, 1>(0, 0).transpose() * del_ptT_T;
        
        assignJacobian(jacobians + 1, del_e_del_T, this->jacsw1, this->jacsw2, this->w1, this->w2, 0, states...);
    }

    return true;
}
}

#endif  // WAVE_PLANE_IMPL_HPP
