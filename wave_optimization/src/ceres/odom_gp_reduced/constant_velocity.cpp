#include <exception>
#include "wave/optimization/ceres/odom_gp_reduced/constant_velocity.hpp"

namespace wave {

/// the parameters are, in order, prev_transform, cur_transform, prev_vel, cur_velocity
bool ConstantVelocityPriorRed::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Mat34> tk_map(parameters[0], 3, 4);
    Eigen::Map<const Mat34> tkp1_map(parameters[1], 3, 4);

    Transformation<Eigen::Map<const Mat34>, true> Tk(tk_map);
    Transformation<Eigen::Map<const Mat34>, true> Tkp1(tkp1_map);

    Eigen::Map<const Vec6> cur_vel(parameters[2]);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> res_map(residuals);

    Mat6 J_left, J_right;
    Vec6 man_diff = Tkp1.manifoldMinusAndJacobian(Tk, &J_left, &J_right);
    
    res_map.block<6,1>(0,0) = man_diff - this->delta_t * cur_vel;

    res_map = this->weight * res_map;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> jac_map(jacobians[0], 6, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = this->weight * J_right;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> jac_map(jacobians[1], 6, 12);
            jac_map.setZero();
            jac_map.block<6,6>(0,0) = this->weight * J_left;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac_map(jacobians[2], 6, 6);
            jac_map = - this->delta_t * this->weight;
        }
    }
    return true;
}

}