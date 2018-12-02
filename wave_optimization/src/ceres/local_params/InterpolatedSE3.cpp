#include "wave/optimization/ceres/local_params/InterpolatedSE3.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace wave {

/**
 * Computes new SE3 element given existing element and small delta applied on the left
 * This is the same as the regular SE3 local parameterization
 *
 * @param x The SE3 element added to. 12-dimensional R, t, stored in following order
 *         R11, R21, R31, R12, R22, R32, R13, R23, R33, Tx, Ty, Tz
 * @param delta se3 element to add to the SE3 element. Stored in
 *        v1, v2, v3, p1, p2, p3. p is the translational portion
 * @param x_plus_delta exp(delta)*x
 * @return true if successful, false if not
 */

bool InterpolatedSE3::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Vec6 delta_vec;
    delta_vec << delta[0],delta[1],delta[2],delta[3],delta[4],delta[5];

    Mat4 T;
    T << x[0], x[3], x[6], x[9],
            x[1], x[4], x[7], x[10],
            x[2], x[5], x[8], x[11],
            0,    0,    0,     1;

    Transformation<Eigen::Matrix<double, 3, 4>> transform;
    transform.setFromMatrix(T);
    transform.manifoldPlus(delta_vec);
    auto R = transform.getRotationMatrix();
    auto trans = transform.getTranslation();

    x_plus_delta[0] = R(0,0);
    x_plus_delta[1] = R(1,0);
    x_plus_delta[2] = R(2,0);
    x_plus_delta[3] = R(0,1);
    x_plus_delta[4] = R(1,1);
    x_plus_delta[5] = R(2,1);
    x_plus_delta[6] = R(0,2);
    x_plus_delta[7] = R(1,2);
    x_plus_delta[8] = R(2,2);
    x_plus_delta[9] = trans(0);
    x_plus_delta[10] = trans(1);
    x_plus_delta[11] = trans(2);

    return true;
}


/**
 * This jacobian is a bit non-standard because it is not the jacobian of the operation
 * above: dX is the delta added above and dX' is the delta added to the interpolated
 * transform. This jacobian is wrt dX'
 * @param x The SE3 object
 * @param jacobian
 * @return
 */
bool InterpolatedSE3::ComputeJacobian(const double *x, double *jacobian) const {
    Mat4 T;
    T << x[0], x[3], x[6], x[9],
            x[1], x[4], x[7], x[10],
            x[2], x[5], x[8], x[11],
            0,    0,    0,     1;

    Transformation<Eigen::Matrix<double, 3, 4>> transform;
    transform.setFromMatrix(T);
    Vec6 epsilon = transform.logMap();

    Eigen::Matrix<double, 12, 6> J_lift;
    J_lift <<    0,  x[2],  -x[1], 0, 0, 0, //
             -x[2],     0,   x[0], 0, 0, 0, //
              x[1], -x[0],      0, 0, 0, 0, //
                 0,  x[5],  -x[4], 0, 0, 0, //
             -x[5],     0,   x[3], 0, 0, 0, //
              x[4], -x[3],      0, 0, 0, 0, //
                 0,  x[8],  -x[7], 0, 0, 0, //
             -x[8],     0,   x[6], 0, 0, 0, //
              x[7], -x[6],      0, 0, 0, 0, //
                 0, x[11], -x[10], 1, 0, 0, //
            -x[11],     0,   x[9], 0, 1, 0, //
             x[10], -x[9],      0, 0, 0, 1; //

    Mat6 J_interpolated;
    Transformation<Eigen::Matrix<double, 3, 4>>::Jinterpolated(epsilon, *(this->alpha), J_interpolated);

    Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobian, 12, 6) = J_lift * J_interpolated;

    return true;
}

}