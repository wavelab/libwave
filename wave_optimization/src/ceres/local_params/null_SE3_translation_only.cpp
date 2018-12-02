#include <algorithm>

#include <unsupported/Eigen/MatrixFunctions>
#include "wave/optimization/ceres/local_params/null_SE3_translation_only.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace wave {

/**
 * Computes new SE3 element given existing element and small delta applied on the left
 *
 * @param x The SE3 element added to. 12-dimensional R, t, stored in following order
 *         R11, R21, R31, R12, R22, R32, R13, R23, R33, Tx, Ty, Tz
 * @param delta translational se3 element to add to the SE3 element. Stored in
 *        p1, p2, p3.
 * @param x_plus_delta exp(delta)*x
 * @return true if successful, false if not
 */

bool NullSE3TranslationParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Vec3> delta_vec(delta);

    Eigen::Map<const Mat34> xmap(x);
    
    Eigen::Map<Mat34> xpdmap(x_plus_delta);

    xpdmap.block<3,3>(0,0).noalias() = xmap.block<3,3>(0,0);
    xpdmap.block<3,1>(0,3).noalias() = xmap.block<3,1>(0,3) + delta_vec;

    return true;
}

/**
 * Computes Bogus lift Jacobian
 * The lift jacobian is a 12x3, with the top 3x3 0s, the bottom half 0s, and the top mid part identity
 *
 * @return true always
 */

bool NullSE3TranslationParameterization::ComputeJacobian(const double *, double *jacobian) const {
    std::fill_n(jacobian, 36, 0.0);
    jacobian[9] = 1;
    jacobian[13] = 1;
    jacobian[17] = 1;

    return true;
}
}
