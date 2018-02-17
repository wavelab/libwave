#include <algorithm>

#include <unsupported/Eigen/MatrixFunctions>
#include "wave/optimization/ceres/local_params/null_SE3_parameterization.hpp"
#include "wave/geometry/transformation.hpp"

namespace wave {

/**
 * Computes new SE3 element given existing element and small delta applied on the left
 *
 * @param x The SE3 element added to. 12-dimensional R, t, stored in following order
 *         R11, R21, R31, R12, R22, R32, R13, R23, R33, Tx, Ty, Tz
 * @param delta se3 element to add to the SE3 element. Stored in
 *        v1, v2, v3, p1, p2, p3. p is the translational portion
 * @param x_plus_delta exp(delta)*x
 * @return true if successful, false if not
 */

bool NullSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Vec6> delta_vec(delta);

    auto x_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(x, 3, 4);
    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>, true> start(x_ptr);

    auto xpd_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(x_plus_delta, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>, true> transform(xpd_ptr);

    transform.deepCopy(start);
    transform.manifoldPlus(delta_vec);
    transform.normalizeMaybe(1e-5);

    return true;
}

/**
 * Computes Bogus lift Jacobian
 * The lift jacobian is a 12x6, with the top half identity and the bottom half 0
 *
 * @return true always
 */

bool NullSE3Parameterization::ComputeJacobian(const double *, double *jacobian) const {
    std::fill_n(jacobian, 72, 0.0);
    jacobian[0] = 1;
    jacobian[7] = 1;
    jacobian[14] = 1;
    jacobian[21] = 1;
    jacobian[28] = 1;
    jacobian[35] = 1;

    return true;
}
}
