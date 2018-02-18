//
// Created by ben on 2/17/18.
//

#ifndef WAVE_CONSTANT_VELOCITY_COUPLED_IMPL_HPP
#define WAVE_CONSTANT_VELOCITY_COUPLED_IMPL_HPP

#include "wave/optimization/ceres/odom_gp_coupled_states/constant_velocity.hpp"

namespace wave {

/// the parameters are, in order, cur_velocity, poses 0 to N
template<int DIM>
bool
ConstantVelocityPriorCoupled<DIM>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    auto tk_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0] + 6 + 12*this->idx_k, 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0] + 6 + 12*(this->idx_k + 1), 3, 4);

    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> Tk(tk_ptr);
    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> Tkp1(tkp1_ptr);

    Eigen::Map<const Vec6> cur_vel(parameters[0]);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> res_map(residuals);

    Mat6 J_left, J_right;
    Vec6 man_diff = Tkp1.manifoldMinusAndJacobian(Tk, J_left, J_right);

    res_map = this->weight * (man_diff - this->delta_t * cur_vel);

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, DIM, Eigen::RowMajor>> jac_map(jacobians[0], 6, DIM);
            jac_map.setZero();

            // Velocity is always at the front
            jac_map.template block<6, 6>(0, 0) = -this->delta_t * this->weight;
            // Tk
            jac_map.template block<6, 6>(0, 6 + 12*this->idx_k) = this->weight * J_right;
            // Tkp1
            jac_map.template block<6, 6>(0, 6 + 12*(this->idx_k + 1)) = this->weight * J_left;
        }
    }
    return true;
}

}

#endif //WAVE_CONSTANT_VELOCITY_COUPLED_IMPL_HPP
