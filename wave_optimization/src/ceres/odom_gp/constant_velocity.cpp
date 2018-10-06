#include <exception>
#include "wave/optimization/ceres/odom_gp/constant_velocity.hpp"

namespace wave {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Mat34> tk_map(parameters[0], 3, 4);
    Eigen::Map<const Mat34> tkp1_map(parameters[1], 3, 4);

    Transformation<Eigen::Map<const Mat34>, true> Tk(tk_map);
    Transformation<Eigen::Map<const Mat34>, true> Tkp1(tkp1_map);

    Eigen::Map<const Vec6> prev_vel(parameters[2]);
    Eigen::Map<const Vec6>  cur_vel(parameters[3]);

    Eigen::Map<Eigen::Matrix<double, 12, 1>> res_map(residuals);

    Mat6 J_left, J_right;
    Vec6 man_diff = Tkp1.manifoldMinusAndJacobian(Tk, &J_left, &J_right);
    
    res_map.block<6,1>(0,0) = man_diff - this->delta_t * prev_vel;
    res_map.block<6,1>(6,0) = J_left * cur_vel - prev_vel;

    res_map = this->weight * res_map;

    if (jacobians) {
        Mat6 skew;
        skew << (man_diff(1) * cur_vel(1)) * 0.08333333333333333333 + (man_diff(2) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(1)) * 0.08333333333333333333 - cur_vel(2) * 0.5 -
                (man_diff(1) * cur_vel(0)) * 0.1666666666666666666666,
                cur_vel(1) * 0.5 + (man_diff(0) * cur_vel(2)) * 0.08333333333333333333 -
                (man_diff(2) * cur_vel(0)) * 0.1666666666666666666666,
                0, 0, 0, cur_vel(2) * 0.5 - (man_diff(0) * cur_vel(1)) * 0.1666666666666666666666 +
                         (man_diff(1) * cur_vel(0)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(0)) * 0.08333333333333333333 + (man_diff(2) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(1) * cur_vel(2)) * 0.08333333333333333333 - cur_vel(0) * 0.5 -
                (man_diff(2) * cur_vel(1)) * 0.1666666666666666666666,
                0, 0, 0, (man_diff(2) * cur_vel(0)) * 0.08333333333333333333 - (man_diff(0) * cur_vel(2)) * 0.1666666666666666666666 -
                         cur_vel(1) * 0.5,
                cur_vel(0) * 0.5 - (man_diff(1) * cur_vel(2)) * 0.1666666666666666666666 +
                (man_diff(2) * cur_vel(1)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(0)) * 0.08333333333333333333 + (man_diff(1) * cur_vel(1)) * 0.08333333333333333333, 0, 0, 0,
                (man_diff(1) * cur_vel(4)) * 0.08333333333333333333 + (man_diff(4) * cur_vel(1)) * 0.08333333333333333333 +
                (man_diff(2) * cur_vel(5)) * 0.08333333333333333333 + (man_diff(5) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(4)) * 0.08333333333333333333 - cur_vel(5) * 0.5 -
                (man_diff(1) * cur_vel(3)) * 0.1666666666666666666666 + (man_diff(3) * cur_vel(1)) * 0.08333333333333333333 -
                (man_diff(4) * cur_vel(0)) * 0.1666666666666666666666,
                cur_vel(4) * 0.5 + (man_diff(0) * cur_vel(5)) * 0.08333333333333333333 -
                (man_diff(2) * cur_vel(3)) * 0.1666666666666666666666 + (man_diff(3) * cur_vel(2)) * 0.08333333333333333333 -
                (man_diff(5) * cur_vel(0)) * 0.1666666666666666666666,
                (man_diff(1) * cur_vel(1)) * 0.08333333333333333333 + (man_diff(2) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(1)) * 0.08333333333333333333 - cur_vel(2) * 0.5 -
                (man_diff(1) * cur_vel(0)) * 0.1666666666666666666666,
                cur_vel(1) * 0.5 + (man_diff(0) * cur_vel(2)) * 0.08333333333333333333 -
                (man_diff(2) * cur_vel(0)) * 0.1666666666666666666666,
                cur_vel(5) * 0.5 - (man_diff(0) * cur_vel(4)) * 0.1666666666666666666666 +
                (man_diff(1) * cur_vel(3)) * 0.08333333333333333333 - (man_diff(3) * cur_vel(1)) * 0.1666666666666666666666 +
                (man_diff(4) * cur_vel(0)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(3)) * 0.08333333333333333333 + (man_diff(3) * cur_vel(0)) * 0.08333333333333333333 +
                (man_diff(2) * cur_vel(5)) * 0.08333333333333333333 + (man_diff(5) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(1) * cur_vel(5)) * 0.08333333333333333333 - cur_vel(3) * 0.5 -
                (man_diff(2) * cur_vel(4)) * 0.1666666666666666666666 + (man_diff(4) * cur_vel(2)) * 0.08333333333333333333 -
                (man_diff(5) * cur_vel(1)) * 0.1666666666666666666666,
                cur_vel(2) * 0.5 - (man_diff(0) * cur_vel(1)) * 0.1666666666666666666666 +
                (man_diff(1) * cur_vel(0)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(0)) * 0.08333333333333333333 + (man_diff(2) * cur_vel(2)) * 0.08333333333333333333,
                (man_diff(1) * cur_vel(2)) * 0.08333333333333333333 - cur_vel(0) * 0.5 -
                (man_diff(2) * cur_vel(1)) * 0.1666666666666666666666,
                (man_diff(2) * cur_vel(3)) * 0.08333333333333333333 - (man_diff(0) * cur_vel(5)) * 0.1666666666666666666666 -
                cur_vel(4) * 0.5 - (man_diff(3) * cur_vel(2)) * 0.1666666666666666666666 +
                (man_diff(5) * cur_vel(0)) * 0.08333333333333333333,
                cur_vel(3) * 0.5 - (man_diff(1) * cur_vel(5)) * 0.1666666666666666666666 +
                (man_diff(2) * cur_vel(4)) * 0.08333333333333333333 - (man_diff(4) * cur_vel(2)) * 0.1666666666666666666666 +
                (man_diff(5) * cur_vel(1)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(3)) * 0.08333333333333333333 + (man_diff(3) * cur_vel(0)) * 0.08333333333333333333 +
                (man_diff(1) * cur_vel(4)) * 0.08333333333333333333 + (man_diff(4) * cur_vel(1)) * 0.08333333333333333333,
                (man_diff(2) * cur_vel(0)) * 0.08333333333333333333 - (man_diff(0) * cur_vel(2)) * 0.1666666666666666666666 -
                cur_vel(1) * 0.5,
                cur_vel(0) * 0.5 - (man_diff(1) * cur_vel(2)) * 0.1666666666666666666666 +
                (man_diff(2) * cur_vel(1)) * 0.08333333333333333333,
                (man_diff(0) * cur_vel(0)) * 0.08333333333333333333 + (man_diff(1) * cur_vel(1)) * 0.08333333333333333333;
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[0], 12, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = J_right;
            jac_map.block<6,6>(6,0) = skew * J_right;
            jac_map.block<12, 6>(0,0) = this->weight * jac_map.block<12, 6>(0,0);
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> jac_map(jacobians[1], 12, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = J_left;
            jac_map.block<6,6>(6,0) = skew * J_left;
            jac_map.block<12, 6>(0,0) = this->weight * jac_map.block<12, 6>(0,0);
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[2], 12, 6);
            jac_map.block<6, 6>(0,0) = - this->delta_t * Mat6::Identity();
            jac_map.block<6, 6>(6,0) = - Mat6::Identity();
            jac_map = this->weight * jac_map;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> jac_map(jacobians[3], 12, 6);
            jac_map.block<6,6>(6,0) = J_left;
            jac_map.block<6,6>(0,0).setZero();
            jac_map = this->weight * jac_map;
        }
    }
    return true;
}

}