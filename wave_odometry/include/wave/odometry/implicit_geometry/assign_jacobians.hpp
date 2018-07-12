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

namespace wave {

// Base case
template<typename Derived>
inline void assignJacobian(double **jacobian,
                    const Eigen::MatrixBase<Derived> &del_e_del_diff,
                    const VecE<Eigen::Tensor<double, 3>> *jacs,
                    int pt_id,
                    int state_idx,
                    int state_dim) {
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            jac(jacobian[state_idx], del_e_del_diff.rows(), state_dim);

    int offset = pt_id * jacs->at(state_idx).dimension(0) * jacs->at(state_idx).dimension(1);
    Eigen::Map<const MatX> precalc(jacs->at(state_idx).data() + offset, del_e_del_diff.cols(), state_dim);

    jac = del_e_del_diff * precalc;
}

// Recurse through parameter pack
template<typename Derived, typename... States>
inline void assignJacobian(double **jacobian,
                    const Eigen::MatrixBase<Derived> &del_e_del_diff,
                    const VecE<Eigen::Tensor<double, 3>> *jacs,
                    int pt_id,
                    int state_idx,
                    int state_dim,
                    States... states) {
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            jac(jacobian[state_idx], del_e_del_diff.rows(), state_dim);

    int offset = pt_id * jacs->at(state_idx).dimension(0) * jacs->at(state_idx).dimension(1);
    Eigen::Map<const MatX> precalc(jacs->at(state_idx).data() + offset, del_e_del_diff.cols(), state_dim);

    jac = del_e_del_diff * precalc;
    assignJacobian(jacobian, del_e_del_diff, jacs, pt_id, state_idx + 1, states...);
};

}

#endif //WAVE_ASSIGN_JACOBIANS_HPP
