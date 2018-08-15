#ifndef WAVE_IMPLICIT_PLANE_IMPL_HPP
#define WAVE_IMPLICIT_PLANE_IMPL_HPP

#include "../implicit_plane.hpp"

namespace wave {

template<int... states>
bool ImplicitPlaneResidual<states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> error(residuals);
    Eigen::Map<const Vec6> plane(&parameters[0][0]);

    // Calculate error
    auto s_id = this->track->mapping.at(this->pt_id).scan_idx;
    auto pt_idx = this->track->mapping.at(this->pt_id).pt_idx;
    auto t_idx = this->track->mapping.at(this->pt_id).state_id;
    Vec3 pt = (this->feat_points_T->at(s_id).at(this->feat_idx).template block<3, 1>(0, pt_idx).template cast<double>();
    Vec3 diff = pt - plane.block<3, 1>(3, 0);
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
        del_ptT_T << 0, pt(2), -pt(1), 1, 0, 0,
                     -pt(2), 0, pt(0), 0, 1, 0,
                     pt(1), -pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 1, 6> del_e_del_T;
        del_e_del_T = plane.block<3, 1>(0, 0).transpose() * del_ptT_T;

        float pttime = this->feat_points->at(s_id).at(this->feat_idx)(3, pt_idx);
        assignJacobian(jacobians + 1, del_e_del_T, this->track->jacs->at(t_idx), pttime, 0, states...);
    }

    return true;
}

}

#endif //WAVE_IMPLICIT_PLANE_IMPL_HPP
