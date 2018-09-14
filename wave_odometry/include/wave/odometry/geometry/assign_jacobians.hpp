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
                           const VecE<const MatX*> &jacsw1,
                           const VecE<const MatX*> &jacsw2,
                           double w1,
                           double w2,
                           uint32_t state_idx,
                           int state_dim) {
    if (jacobian[state_idx]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jac(
          jacobian[state_idx], del_e_del_T.rows(), state_dim);

        auto true_dim = jacsw1.at(state_idx)->cols();
        if (state_dim != true_dim) {
            jac.block(0, 0, del_e_del_T.rows(), true_dim).noalias() =
              del_e_del_T * (w1 * *(jacsw1.at(state_idx)) + w2 * *(jacsw2.at(state_idx))).eval();
            jac.block(0, true_dim, del_e_del_T.rows(), state_dim - true_dim).setZero();
        } else {
            jac.noalias() = del_e_del_T * (w1 * *(jacsw1.at(state_idx)) + w2 * *(jacsw2.at(state_idx))).eval();
        }
    }
}

// Recurse through parameter pack
template <typename Derived, typename... States>
inline void assignJacobian(double **jacobian,
                           const Eigen::MatrixBase<Derived> &del_e_del_T,
                           const VecE<const MatX*> &jacsw1,
                           const VecE<const MatX*> &jacsw2,
                           double w1,
                           double w2,
                           uint32_t state_idx,
                           int state_dim,
                           States... states) {
    assignJacobian(jacobian, del_e_del_T, jacsw1, jacsw2, w1, w2, state_idx, state_dim);
    assignJacobian(jacobian, del_e_del_T, jacsw1, jacsw2, w1, w2, state_idx + 1, states...);
};
}

#endif  // WAVE_ASSIGN_JACOBIANS_HPP
