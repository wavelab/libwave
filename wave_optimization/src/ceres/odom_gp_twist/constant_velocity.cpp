#include <exception>
#include "wave/optimization/ceres/odom_gp_twist/constant_velocity.hpp"

namespace wave_optimization {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const wave::Vec12> state_k_map(parameters[0]);
    Eigen::Map<const wave::Vec12> state_tkp1_map(parameters[1]);

    Eigen::Map<Eigen::Matrix<double, 12, 1>> res_map(residuals);
    res_map.block<6, 1>(0, 0) = this->E_op.block<6, 1>(0, 0) + state_tkp1_map.block<6, 1>(0, 0) -
                                state_k_map.block<6, 1>(0, 0) - this->delta_t * state_k_map.block<6, 1>(6, 0);
    res_map.block<6, 1>(6, 0) =
      this->E_op.block<6, 1>(6, 0) + state_tkp1_map.block<6, 1>(6, 0) - state_k_map.block<6, 1>(6, 0);
    res_map = this->weight * res_map;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[0], 12, 12);
            // Pose component
            jac_map.block<6, 6>(0, 0) = -wave::Mat6::Identity();
            jac_map.block<6, 6>(6, 0).setZero();
            // Velocity component
            jac_map.block<6, 6>(0, 6) = -this->delta_t * wave::Mat6::Identity();
            jac_map.block<6, 6>(6, 6) = -wave::Mat6::Identity();

            jac_map = this->weight * jac_map;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[1], 12, 12);
            // Pose component
            jac_map.block<6, 6>(0, 0).setIdentity();
            jac_map.block<6, 6>(6, 0).setZero();
            //Velocity Component
            jac_map.block<6, 6>(0, 6).setZero();
            jac_map.block<6, 6>(6, 6).setIdentity();

            jac_map = this->weight * jac_map;
        }
    }
    return true;
}
}