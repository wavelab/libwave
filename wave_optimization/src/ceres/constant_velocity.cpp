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

    Mat6 J_left, J_right;
    Vec6 man_minus = cur_transform.manifoldMinusAndJacobian(prev_transform, J_left, J_right);
    res_map.block<6,1>(0,0) = man_minus - this->delta_t * prev_vel;

    res_map.block<6,1>(6,0) = J_left * cur_vel - prev_vel;

    res_map = this->weight * res_map;
    Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals, 12, 1) = res_map;

    if (jacobians) {
        Mat6 skew = Transformation::skewSymmetric6(cur_vel);
        if (jacobians[0]) {
            this->Jr_Ti.block<6,6>(0,0) = J_right;
            this->Jr_Ti.block<6,6>(6,0) = 0.5 * skew * J_right;
            this->Jr_Ti.block<12, 6>(0,0) = this->weight * this->Jr_Ti.block<12, 6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[0], 12, 12) = this->Jr_Ti;
        }
        if (jacobians[1]) {
            this->Jr_Tip1.block<6,6>(0,0) = J_left;
            this->Jr_Tip1.block<6,6>(6,0) = 0.5 * skew * J_left;
            this->Jr_Tip1.block<12, 6>(0,0) = this->weight * this->Jr_Tip1.block<12, 6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[1], 12, 12) = this->Jr_Tip1;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[2], 12, 6) = this->Jr_wi;
        }
        if (jacobians[3]) {
            this->Jr_wip1.block<6,6>(6,0) = this->Jr_Tip1.block<6,6>(0,0);
            this->Jr_wip1.block<12, 6>(0,0) = this->weight * this->Jr_wip1.block<12, 6>(0,0);

            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[3], 12, 6) = this->Jr_wip1;
        }
    }
    return true;
}

}