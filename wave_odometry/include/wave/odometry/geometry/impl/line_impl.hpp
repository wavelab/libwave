#ifndef WAVE_LINE_IMPL_HPP
#define WAVE_LINE_IMPL_HPP

#include <wave/odometry/geometry/assign_jacobians.hpp>
#include "wave/odometry/geometry/line.hpp"

namespace wave {

template <int... states>
bool LineResidual<states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);
    Eigen::Map<const Vec6> line(parameters[0]);

    auto normal = line.block<3, 1>(0, 0);
    auto avg = line.block<3, 1>(3, 0);

    // Calculate error
    auto s_id = this->track->mapping.at(this->pt_id).scan_idx;

    Vec3 pt = this->feat_points_T->at(s_id)
                 .at(this->feat_idx)
                 .template block<3, 1>(0, this->track->mapping.at(this->pt_id).pt_idx)
                .template cast<double>();
    Vec3 diff = pt - avg;
    double dp = (normal.transpose() * diff)(0);
    error = diff - dp * normal;

    if (std::isnan(residuals[0])) {
        throw std::runtime_error("nan in line residual, probably due to incorrect point index");
    }

    if (jacobians) {
        Eigen::Matrix<double, 3, 3> del_e_del_diff;
        del_e_del_diff = Mat3::Identity() - normal * normal.transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> line_jac(jacobians[0]);

            // jacobian wrt normal
            line_jac.block<3, 3>(0, 0) = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            line_jac.block<3, 3>(0, 3) = -del_e_del_diff;
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        del_ptT_T << 0, pt(2), -pt(1), 1, 0, 0, -pt(2), 0, pt(0), 0, 1, 0, pt(1), -pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 3, 6> del_e_del_T = del_e_del_diff * del_ptT_T;
        
        assignJacobian(jacobians + 1, del_e_del_T, *(this->jacs), this->w1, this->w2, this->jac_index, 0, states...);
    }
    return true;
}
}

#endif  // WAVE_LINE_IMPL_HPP
