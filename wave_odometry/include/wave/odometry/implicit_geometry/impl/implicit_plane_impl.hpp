#ifndef WAVE_IMPLICIT_PLANE_IMPL_HPP
#define WAVE_IMPLICIT_PLANE_IMPL_HPP

#include "../implicit_plane.hpp"

namespace wave {

template<int... states>
bool ImplicitPlaneResidual<states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> error(residuals);
    Eigen::Map<const Vec6> plane(&parameters[0][0]);

    auto normal = plane.block<3, 1>(0,0);
    auto avg = plane.block<3, 1>(3,0);

    // Calculate error
    auto s_id = this->track->mapping.at(this->pt_id).scan_idx;
    Vec3 pt = (this->feat_points->at(s_id).at(this->track->featT_idx).template block<3, 1>(0, this->track->mapping.at(this->pt_id).pt_idx)).template cast<double>();
    Vec3 diff = pt - avg;
    error = diff.transpose() * normal;

    if (jacobians) {
        Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> plane_jac(jacobians[0]);

        Eigen::Matrix<double, 1, 3> del_e_del_diff;
        del_e_del_diff = normal.transpose();

        plane_jac.template block<1, 3>(0, 0) = diff.transpose();
        plane_jac.template block<1, 3>(0, 3) = -normal.transpose();

        for (uint32_t i = 0; i < sizeof...(states); i++) {
            Eigen::Map<Eigen::Matrix<double, 1, get<0>(states...), Eigen::RowMajor>> jac(jacobians[i+1]);
            jac = del_e_del_diff * this->track->jacs.at(this->pt_id).at(i);
        }
    }
    return true;
}

}

#endif //WAVE_IMPLICIT_PLANE_IMPL_HPP
