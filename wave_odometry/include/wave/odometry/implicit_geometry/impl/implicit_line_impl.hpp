#ifndef WAVE_IMPLICIT_LINE_IMPL_HPP
#define WAVE_IMPLICIT_LINE_IMPL_HPP

#include "../implicit_line.hpp"

namespace wave {

template<int cnt, int state_dim, int... num>
bool ImplicitLineResidual<cnt, state_dim, num...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, cnt*3, 1>> error(residuals);
    Eigen::Map<const Vec3> normal(&parameters[0][0]);
    auto cnt_d = static_cast<double>(cnt);

    if (jacobians) {
        std::vector<Eigen::Map<Eigen::Matrix<double, cnt*3, state_dim, Eigen::RowMajor>>> state_jacs;
        Eigen::Map<Eigen::Matrix<double, cnt*3, 3, Eigen::RowMajor>> normal_jac(jacobians[0]);

        for (uint32_t i = 0; i < sizeof...(num); i++) {
            state_jacs.emplace_back(Eigen::Map<Eigen::Matrix<double, cnt*3, state_dim, Eigen::RowMajor>>(jacobians[i+1]));
            state_jacs.at(i).template setZero();
        }
        // Calculate error and jacobians
        for (uint32_t i = 0; i < this->track.mapping.size(); i++) {
            Vec3 diff = (this->feat_points->at(this->track.mapping.at(i).scan_idx).at(this->track.featT_idx).block<3, 1>(0, this->track.mapping.at(i).pt_idx) -
                         this->avg_points->at(this->track.featT_idx).block<3, 1>(0, this->track.ave_pt_idx)).template cast<double>();
            double dp = (normal.transpose() * diff)(0);
            error.template block<3, 1>(i * 3, 0) = diff - dp * normal;

            double nm1on = (cnt_d - 1.0) / cnt_d;
            double oon = -1.0 / cnt_d;

            normal_jac.template block<3, 3>(i*3, 0) = - normal * diff.transpose() - dp * Mat3::Identity();
            Eigen::Matrix<double, 3, 3> del_e_del_diff;
            del_e_del_diff = Mat3::Identity() - normal * normal.transpose();
            state_jacs.at(this->track.p_states.at(i)).template block<3, state_dim>(i*3, 0) += del_e_del_diff * nm1on * this->track.prev_jac.at(i);
            state_jacs.at(this->track.n_states.at(i)).template block<3, state_dim>(i*3, 0) += del_e_del_diff * nm1on * this->track.next_jac.at(i);
            for (uint32_t j = 0; j < this->track.mapping.size(); j++) {
                if (i == j) {
                    continue;
                }
                state_jacs.at(this->track.p_states.at(j)).template block<3, state_dim>(i*3, 0) += del_e_del_diff * oon * this->track.prev_jac.at(j);
                state_jacs.at(this->track.n_states.at(j)).template block<3, state_dim>(i*3, 0) += del_e_del_diff * oon * this->track.next_jac.at(j);
            }
        }
    } else {
        // Calculate error
        for (uint32_t i = 0; i < this->track.mapping.size(); i++) {
            Vec3 diff = (this->feat_points->at(this->track.mapping.at(i).scan_idx).at(this->track.featT_idx).block<3, 1>(0, this->track.mapping.at(i).pt_idx) -
                    this->avg_points->at(this->track.featT_idx).block<3, 1>(0, this->track.ave_pt_idx)).template cast<double>();

            double dp = (normal.transpose() * diff)(0);
            error.template block<3, 1>(i * 3, 0) = diff - normal * dp;
        }
    }
    return true;
}

}

#endif //WAVE_IMPLICIT_LINE_IMPL_HPP
