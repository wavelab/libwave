#include "wave/optimization/ceres/SE3Parameterization.hpp"

namespace wave {

/**
 * Computes new SE3 element given existing element and small delta applied on the left
 *
 * @param x The SE3 element added to. 12-dimensional R, t, stored in row-major form
 *         aka R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
 * @param delta se3 element to add to the SE3 element. Stored in
 *        p1, p2, p3, v1, v2, v3. p is the translational portion
 * @param x_plus_delta exp(delta)*x
 * @return true if successful, false if not
 */

bool SE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    /**
     * ^ operator on delta
     * 0         -delta[5] delta[4]  delta[0]
     *  delta[5] 0         -delta[3] delta[1]
     * -delta[4] delta[3]  0         delta[2]
     * 0         0         0         1
     */
    Eigen::Matrix4d deltahat, X;
    deltahat << 0, -delta[5], delta[4], delta[0],
            delta[5], 0,    -delta[3], delta[1],
            -delta[4], delta[3],  0,   delta[2],
            0, 0, 0, 1;
    ceres::MatrixRef x_map(x, 3, 4);

    X.block(0,0,3,4) = x_map;
    X(3,0) = 0;
    X(3,1) = 0;
    X(3,2) = 0;
    X(3,3) = 1;

}

}