#include <exception>
#include "wave/optimization/ceres/constant_velocity.hpp"

namespace wave {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Transformation prev_transform, cur_transform;
    prev_transform.getInternalMatrix() = Eigen::Map<const Eigen::Matrix<double, 3, 4>>(parameters[0]);
    cur_transform.getInternalMatrix()  = Eigen::Map<const Eigen::Matrix<double, 3, 4>>(parameters[1]);
    Eigen::Map<const Vec6> prev_vel(parameters[2]);
    Eigen::Map<const Vec6>  cur_vel(parameters[3]);

    Eigen::Matrix<double, 12, 1> g_k, g_kp1, e;

    g_k.block<6,1>(0,0) = (prev_transform * this->inv_prev_transform_prior).logMap();
    g_k.block<6,1>(6,0) = prev_vel - this->prev_vel_prior;

    g_kp1.block<6,1>(0,0) = (cur_transform * this->inv_cur_transform_prior).logMap();
    g_kp1.block<6,1>(6,0) = cur_vel - this->cur_vel_prior;

    e = this->weight * (this->transition_matrix * g_k - g_kp1);
    Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals, 12, 1) = e;

    if (jacobians) {
        /// Going to approximate the derivative of the logmap as identity under the assumption that the
        /// difference between the operating point and the prior is small
        if (jacobians[0]) {
            this->J12.block<12,6>(0,0) = this->weight * this->transition_matrix.block<12,6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[0], 12, 12) = this->J12;
        }
        if (jacobians[1]) {
            this->J12.block<6,6>(0,0) = -Eigen::Matrix<double, 6, 6>::Identity();
            this->J12.block<6,6>(6,0).setZero();

            this->J12.block<12,6>(0,0) = this->weight * this->J12.block<12,6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[1], 12, 12) = this->J12;
        }
        if (jacobians[2]) {
            this->J6 = this->weight * this->transition_matrix.block<12,6>(0,6);
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[2], 12, 6) = this->J6;
        }
        if (jacobians[3]) {
            this->J6 = -this->weight.block<12, 6>(0, 6);
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[3], 12, 6) = this->J6;
        }
    }
    return true;
}

}