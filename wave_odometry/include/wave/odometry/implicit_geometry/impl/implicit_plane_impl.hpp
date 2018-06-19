#ifndef WAVE_IMPLICIT_PLANE_IMPL_HPP
#define WAVE_IMPLICIT_PLANE_IMPL_HPP

#include "../implicit_plane.hpp"

namespace wave {

template<int cnt, int state_dim, int... num>
bool ImplicitPlaneResidual<cnt, state_dim, num...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, cnt, 1>> error(residuals);
    Eigen::Map<const Vec3> normal(&parameters[0][0]);

    //Calculate average point
    Vec3 avg;
    avg.setZero();
    for (const auto &pt : this->pts.tpts) {
        avg = avg + pt;
    }
    double cnt_d = static_cast<double>(cnt);
    avg = avg / cnt_d;

    if (jacobians) {
        std::vector<Eigen::Map<Eigen::Matrix<double, cnt, state_dim, Eigen::RowMajor>>> state_jacs;
        Eigen::Map<Eigen::Matrix<double, cnt, 3, Eigen::RowMajor>> normal_jac(jacobians[0]);

        for (uint32_t i = 0; i < sizeof...(num); i++) {
            state_jacs.emplace_back(Eigen::Map<Eigen::Matrix<double, cnt, state_dim, Eigen::RowMajor>>(jacobians[i+1]));
            state_jacs.at(i).template setZero();
        }
        // Calculate error and jacobians
        for (uint32_t i = 0; i < this->pts.tpts.size(); i++) {
            Vec3 diff = this->pts.tpts.at(i) - avg;
            double nm1on = (cnt_d - 1.0) / cnt_d;
            double oon = 1.0 / cnt_d;
            error(i, 0) = diff.transpose() * normal;
            normal_jac.template block<1, 3>(i, 0) = diff.transpose();
            Eigen::Matrix<double, 1, 3> del_e_del_diff;
            del_e_del_diff = normal.transpose();
            state_jacs.at(this->pts.p_states.at(i)).template block<1, state_dim>(i, 0) += del_e_del_diff * nm1on * this->pts.prev_jac.at(i);
            state_jacs.at(this->pts.n_states.at(i)).template block<1, state_dim>(i, 0) += del_e_del_diff * nm1on * this->pts.next_jac.at(i);
            for (uint32_t j = 0; j <this->pts.tpts.size(); j++) {
                if (i == j) {
                    continue;
                }
                state_jacs.at(this->pts.p_states.at(j)).template block<1, state_dim>(i, 0) -= del_e_del_diff * oon * this->pts.prev_jac.at(j);
                state_jacs.at(this->pts.n_states.at(j)).template block<1, state_dim>(i, 0) -= del_e_del_diff * oon * this->pts.next_jac.at(j);
            }
        }
    } else {
        // Calculate error
        for (uint32_t i = 0; i < this->pts.tpts.size(); i++) {
            auto diff = this->pts.tpts.at(i) - avg;
            error(i, 0) = diff.transpose() * normal;
        }
    }
    return true;
}

}

#endif //WAVE_IMPLICIT_PLANE_IMPL_HPP
