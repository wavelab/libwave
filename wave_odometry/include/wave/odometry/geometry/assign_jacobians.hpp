/**
 * To iterate through a parameter pack, need to use recursion
 * https://stackoverflow.com/questions/7230621/how-can-i-iterate-over-a-packed-variadic-template-argument-list#7232968
 *
 * For getting a value from a variable template parameter pack
 * https://stackoverflow.com/questions/24710389/how-to-extract-a-value-from-a-variadic-template-parameter-pack-by-index
 *
 * This is pretty neat, templates are recursively instantiated when compiling
 */

#ifndef WAVE_ASSIGN_JACOBIANS_HPP
#define WAVE_ASSIGN_JACOBIANS_HPP

#include <Eigen/Eigen>
#include "wave/odometry/feature_track.hpp"
#include "wave/utils/types.hpp"

namespace wave {

template <size_t N, typename... Args>
decltype(auto) magic_get(Args&&... as) noexcept {
    return std::get<N>(std::forward_as_tuple(std::forward<Args>(as)...));
}

// Base case
template <int state_dim, typename Derived>
inline void assignSpecificJacobian(double **jacobian,
                           const Eigen::MatrixBase<Derived> &del_e_del_T,
                           const VecE<const MatX*> &jacsw1,
                           const VecE<const MatX*> &jacsw2,
                           double w1,
                           double w2,
                           uint32_t state_idx) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Eigen::MatrixBase<Derived>)

    const int rows = Eigen::MatrixBase<Derived>::RowsAtCompileTime;

    if (jacobian[state_idx]) {
        Eigen::Map<Eigen::Matrix<double, state_dim, rows>> jac(
                jacobian[state_idx]);

        if constexpr (state_dim > 6) {
            jac.template block<6, rows>(0,0).noalias() = (del_e_del_T * (w1 * *(jacsw1.at(state_idx)) + w2 * *(jacsw2.at(state_idx)))).transpose();
            jac.template block<state_dim - 6, rows>(6, 0).setZero();
        } else {
            jac.noalias() = (del_e_del_T * (w1 * *(jacsw1.at(state_idx)) + w2 * *(jacsw2.at(state_idx)))).transpose();
        }
    }
}

// Recurse through parameter pack
template <int state_dim, int... state_dims, typename Derived>
inline void assignJacobian(double **jacobian,
                           const Eigen::MatrixBase<Derived> &del_e_del_T,
                           const VecE<const MatX*> &jacsw1,
                           const VecE<const MatX*> &jacsw2,
                           double w1,
                           double w2,
                           uint32_t state_idx) {
    assignSpecificJacobian<state_dim>(jacobian, del_e_del_T, jacsw1, jacsw2, w1, w2, state_idx);
    if constexpr (sizeof...(state_dims) > 0) {
        assignJacobian<state_dims...>(jacobian, del_e_del_T, jacsw1, jacsw2, w1, w2, state_idx + 1);
    }
};
}

#endif  // WAVE_ASSIGN_JACOBIANS_HPP
