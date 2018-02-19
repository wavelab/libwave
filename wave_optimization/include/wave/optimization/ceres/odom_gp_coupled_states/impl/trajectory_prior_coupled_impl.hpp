#ifndef WAVE_TRAJECTORY_PRIOR_COUPLED_IMPL_HPP
#define WAVE_TRAJECTORY_PRIOR_COUPLED_IMPL_HPP

#include "wave/optimization/ceres/odom_gp_coupled_states/trajectory_prior_coupled.hpp"

namespace wave {

template<typename T_TYPE, int DIM>
TrajectoryPriorCoupled<T_TYPE, DIM>::~TrajectoryPriorCoupled() {}

template<typename T_TYPE, int DIM>
TrajectoryPriorCoupled<T_TYPE, DIM>::TrajectoryPriorCoupled(Mat12 weight_matrix, const T_TYPE &inv_prior, const Vec6 &twist_prior, const int &idx_k)
: idx_k(idx_k), inv_prior(inv_prior), twist_prior(twist_prior), weight_matrix(weight_matrix) {
    if (!inv_prior.matrix->allFinite()) {
        throw std::invalid_argument("Inverse Prior not finite");
    }
    if (!twist_prior.allFinite()) {
        throw std::invalid_argument("Twist Prior not finite");
    }
    if (!weight_matrix.allFinite()) {
        throw std::invalid_argument("Trajectory Prior weight matrix not finite");
    }
}

template<typename T_TYPE, int DIM>
bool TrajectoryPriorCoupled<T_TYPE, DIM>::Evaluate(double const *const *parameters, double *residuals,
                                                   double **jacobians) const {
    auto tk_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0] + 6 + 12*this->idx_k, 3, 4);
    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> transform(tk_ptr);
    Eigen::Map<const Vec6> twist_map(parameters[0]);

    auto diff = transform * this->inv_prior;
    Eigen::Map<Eigen::Matrix<double, 12, 1>> residual(residuals, 12, 1);
    residual.block<6, 1>(0, 0) = diff.logMap();
    residual.block<6, 1>(6, 0) = twist_map - this->twist_prior;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 12, DIM>> jac_map(jacobians[0]);
            jac_map.setZero();

            // Twist state
            jac_map.template block<12,6>(0,0) = this->weight_matrix.template block<12, 6>(0, 6);
            // Pose state (approximating logmap as zero)
            jac_map.template block<12,6>(0,6 + 12*idx_k) = this->weight_matrix.template block<12, 6>(0,0);
        }
    }
    residual = this->weight_matrix * residual;

    return true;
}

}

#endif //WAVE_TRAJECTORY_PRIOR_COUPLED_IMPL_HPP
