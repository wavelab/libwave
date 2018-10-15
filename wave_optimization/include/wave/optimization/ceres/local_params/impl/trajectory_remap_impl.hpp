#ifndef WAVE_TRAJECTORY_REMAP_IMPL_HPP
#define WAVE_TRAJECTORY_REMAP_IMPL_HPP

#include "wave/optimization/ceres/local_params/trajectory_remap.hpp"

namespace wave {

/**
 * Computes new Trajectory state given existing element, small delta, and a matrix to
 * project delta into a well-conditioned subspace
 *
 * @param x The trajectory state to add to. Stored as a velocity and N poses
 * @param delta trajectory element to add to the state element. Stored in
 *        v1, v2, v3, p1, p2, p3. p is the translational portion
 * @param x_plus_delta exp(this->p_mat * delta)*x
 * @return true if successful, false if not
 */

template<int DIM>
bool TrajectoryParamRemap<DIM>::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, DIM/2 + 3, 1>> delta_vec(delta);
    Eigen::Matrix<double, DIM/2 + 3, 1> remap_delta_vec = this->p_mat * delta_vec;

    Eigen::Map<const Vec6> vel(x);
    Eigen::Map<Vec6> velpd(x_plus_delta);
    velpd = vel + remap_delta_vec.template block<6,1>(0,0);

    int n_pose = (DIM - 6)/12;
    for(int i = 0; i < n_pose; i++) {
        Eigen::Map<const Mat34> x_map(x + 6 + 12*i, 3, 4);
        Eigen::Map<Mat34> xpd_map(x_plus_delta + 6 + 12*i, 3, 4);
        Transformation<Eigen::Map<const Mat34>, true> T(x_map);
        Transformation<Eigen::Map<Mat34>, true> Tpd(xpd_map);

        Tpd = T;
        Tpd.manifoldPlus(remap_delta_vec.template block<6,1>(6 + 6*i, 0));
        Tpd.normalize();
    }

    return true;
}

/**
 * Computes Bogus lift Jacobian
 *
 * Has dimensionality DIM by DIM/2 + 3. Shaped to discard uneeded columns in jacobian
 *
 * @return true always
 */

template<int DIM>
bool TrajectoryParamRemap<DIM>::ComputeJacobian(const double *, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, DIM, DIM/2 + 3, Eigen::RowMajor>> jac_map(jacobian);
    jac_map.setZero();

    jac_map.template block<6,DIM/2 + 3>(0,0) = this->p_mat.template block<6, DIM/2 + 3>(0,0);
    int n_pose = (DIM - 6)/12;
    for(int i = 0; i < n_pose; i++) {
        jac_map.template block<6,DIM/2 + 3>(6 + 12*i, 0) = this->p_mat.template block<6, DIM/2 + 3>(6 + 6*i,0);
    }

    return true;
}

}

#endif //WAVE_TRAJECTORY_REMAP_IMPL_HPP
