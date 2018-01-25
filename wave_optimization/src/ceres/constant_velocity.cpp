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

    Eigen::Map<Eigen::Matrix<double, 12, 1>> res_map(residuals);

    Vec6 man_minus = cur_transform.manifoldMinus(prev_transform);
    res_map.block<6,1>(0,0) = man_minus - this->delta_t * prev_vel;
    auto J_logmap = Transformation::SE3LeftJacobian(man_minus, 0.1).inverse();
    res_map.block<6,1>(6,0) = J_logmap * cur_vel - prev_vel;

    res_map = this->weight * res_map;
    Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals, 12, 1) = res_map;

    if (jacobians) {
        /// Going to approximate the derivative of the logmap as identity under the assumption that the
        /// difference between the operating point and the prior is small
        if (jacobians[0]) {
            this->J12.block<12,6>(0,0) = -this->weight.block<12,6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[0], 12, 12) = this->J12;
        }
        if (jacobians[1]) {
            this->J12.block<12,6>(0,0) = this->weight.block<12,6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[1], 12, 12) = this->J12;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[2], 12, 6) = this->Jr_wi;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[3], 12, 6) = this->Jr_wip1;
        }
    }
    return true;
}

}