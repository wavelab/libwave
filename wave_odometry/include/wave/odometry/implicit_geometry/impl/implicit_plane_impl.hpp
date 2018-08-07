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
    Vec3 pt = (this->feat_points->at(s_id).at(this->feat_idx)->template block<3, 1>(0, this->track->mapping.at(this->pt_id).pt_idx)).template cast<double>();
    Vec3 diff = pt - plane.block<3, 1>(3, 0);
    error = diff.transpose() * plane.block<3, 1>(0, 0);

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> plane_jac(jacobians[0]);

            plane_jac.template block<1, 3>(0, 0) = diff.transpose();
            plane_jac.template block<1, 3>(0, 3) = -plane.block<3, 1>(0, 0).transpose();
        }
        Eigen::Matrix<double, 1, 3> del_e_del_diff;
        del_e_del_diff = plane.block<3, 1>(0, 0).transpose();

        assignJacobian(jacobians + 1, del_e_del_diff, &(this->track->jacs->at(s_id)), this->pt_id, 0, states...);
    }

//    if (residuals[0] > 1.0 || residuals[0] < -1.0) {
//        int voider = 2;
//    }
    return true;
}

}

#endif //WAVE_IMPLICIT_PLANE_IMPL_HPP
