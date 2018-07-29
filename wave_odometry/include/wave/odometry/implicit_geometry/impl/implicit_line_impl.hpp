#ifndef WAVE_IMPLICIT_LINE_IMPL_HPP
#define WAVE_IMPLICIT_LINE_IMPL_HPP

#include <wave/odometry/implicit_geometry/assign_jacobians.hpp>
#include "../implicit_line.hpp"

namespace wave {

template<int... states>
bool ImplicitLineResidual<states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);
    Eigen::Map<const Vec6> line(parameters[0]);

    auto normal = line.block<3, 1>(0,0);
    auto avg = line.block<3, 1>(3,0);

    // Calculate error
    auto s_id = this->track->mapping.at(this->pt_id).scan_idx;

    Vec3 pt = (this->feat_points->at(s_id).at(this->feat_idx)->template block<3, 1>(0, this->track->mapping.at(this->pt_id).pt_idx)).template cast<double>();
    Vec3 diff = pt - avg;
    double dp = (normal.transpose() * diff)(0);
    error = diff - dp * normal;

    if (jacobians) {
        Eigen::Matrix<double, 3, 3> del_e_del_diff;
        del_e_del_diff = Mat3::Identity() - normal * normal.transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> line_jac(jacobians[0]);

            // jacobian wrt normal
            line_jac.block<3, 3>(0,0) = - normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            line_jac.block<3, 3>(0,3) = -del_e_del_diff;
        }

        assignJacobian(jacobians + 1, del_e_del_diff, &(this->track->jacs->at(s_id)), this->pt_id, 0, states...);
    }
    return true;
}

}

#endif //WAVE_IMPLICIT_LINE_IMPL_HPP
