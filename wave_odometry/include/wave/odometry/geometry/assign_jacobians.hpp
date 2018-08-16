/**
 * To iterate through a parameter pack, need to use recursion
 * https://stackoverflow.com/questions/7230621/how-can-i-iterate-over-a-packed-variadic-template-argument-list#7232968
 *
 * This is pretty neat, templates are recursively instantiated when compiling
 */

#ifndef WAVE_ASSIGN_JACOBIANS_HPP
#define WAVE_ASSIGN_JACOBIANS_HPP

#include <Eigen/Eigen>
#include "wave/odometry/feature_track.hpp"
#include "wave/utils/types.hpp"

namespace wave {

// Base case
template <typename Derived>
inline void assignJacobian(double **jacobian,
                           const Eigen::MatrixBase<Derived> &del_e_del_T,
                           const Vec<VecE<MatX>> &jacs,
                           double w1,
                           double w2,
                           uint32_t jac_index,
                           int state_idx,
                           int state_dim) {
    if (jacobian[state_idx]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac(
          jacobian[state_idx], del_e_del_T.rows(), state_dim);

        auto true_dim = jacs.at(state_idx).at(jac_index).cols();
        if (state_dim != true_dim) {
            jac.block(0, 0, del_e_del_T.rows(), true_dim) =
              del_e_del_T * (w1 * jacs.at(state_idx).at(jac_index) + w2 * jacs.at(state_idx).at(jac_index + 1));
            jac.block(0, true_dim, del_e_del_T.rows(), state_dim - true_dim).setZero();
        } else {
            jac = del_e_del_T * (w1 * jacs.at(state_idx).at(jac_index) + w2 * jacs.at(state_idx).at(jac_index + 1));
        }
    }
}

// Recurse through parameter pack
template <typename Derived, typename... States>
inline void assignJacobian(double **jacobian,
                           const Eigen::MatrixBase<Derived> &del_e_del_T,
                           const Vec<VecE<MatX>> &jacs,
                           double w1,
                           double w2,
                           uint32_t jac_index,
                           int state_idx,
                           int state_dim,
                           States... states) {
    if (jacobian[state_idx]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac(
          jacobian[state_idx], del_e_del_T.rows(), state_dim);

        auto true_dim = jacs.at(state_idx).at(jac_index).cols();
        if (state_dim != true_dim) {
            jac.block(0, 0, del_e_del_T.rows(), true_dim) =
              del_e_del_T * (w1 * jacs.at(state_idx).at(jac_index) + w2 * jacs.at(state_idx).at(jac_index + 1));
            jac.block(0, true_dim, del_e_del_T.rows(), state_dim - true_dim).setZero();
        } else {
            jac = del_e_del_T * (w1 * jacs.at(state_idx).at(jac_index) + w2 * jacs.at(state_idx).at(jac_index + 1));
        }
    }

    assignJacobian(jacobian, del_e_del_T, jacs, w1, w2, jac_index, state_idx + 1, states...);
};
}

#endif  // WAVE_ASSIGN_JACOBIANS_HPP
