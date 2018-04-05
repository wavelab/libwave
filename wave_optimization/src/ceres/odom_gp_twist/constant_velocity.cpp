#include <exception>
#include "wave/optimization/ceres/odom_gp_twist/constant_velocity.hpp"

namespace wave_optimization {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const wave::Vec6> tk_map(parameters[0]);
    Eigen::Map<const wave::Vec6> tkp1_map(parameters[1]);
    Eigen::Map<const wave::Vec6> prev_vel(parameters[2]);
    Eigen::Map<const wave::Vec6>  cur_vel(parameters[3]);

    Eigen::Map<Eigen::Matrix<double, 12, 1>> res_map(residuals);
    res_map.block<6,1>(0,0) = this->E_op.block<6,1>(0,0) + tkp1_map - tk_map - this->delta_t * prev_vel;
    res_map.block<6,1>(6,0) = this->E_op.block<6,1>(6,0) + cur_vel - prev_vel;
    res_map = this->weight * res_map;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[0], 12, 6);
            jac_map.block<6,6>(0,0).setIdentity();
            jac_map.block<6,6>(0,0) *= -1;
            jac_map.block<6,6>(6,0).setZero();
            jac_map = this->weight * jac_map;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[1], 12, 6);
            jac_map.setZero();
            jac_map.block<6,6>(0,0).setIdentity();
            jac_map.block<6,6>(6,0).setZero();
            jac_map = this->weight * jac_map;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[2], 12, 6);
            jac_map.block<6, 6>(0,0) = - this->delta_t * wave::Mat6::Identity();
            jac_map.block<6, 6>(6,0) = - wave::Mat6::Identity();
            jac_map = this->weight * jac_map;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[3], 12, 6);
            jac_map.block<6,6>(0,0).setZero();
            jac_map.block<6,6>(6,0).setIdentity();
            jac_map = this->weight * jac_map;
        }
    }
    return true;
}

}