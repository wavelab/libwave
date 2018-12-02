// for matrix square root
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"
#include "wave/geometry_og/transformation.hpp"

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

bool SE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
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

    // Check if R has strayed too far outside SO(3)
    // and if so normalize
    if ((R.matrix().determinant() - 1) > 1e-5) {
        decltype(R) temp = R * R.transpose();
        R = temp.sqrt().inverse() * R;
    }

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
 * Computes new jacobian of box+ wrt the perturbation. Jacobian is taken where delta = 0
 *
 * @param x The SE3 element added to. 12-dimensional R, t, stored in column-major form
 *         aka R11, R21, R31, R12, R22, R32, R13, R23, R33, Tx, Ty, Tz
 * @param jacobian: of box-plus Stored in row major form
 *        Row order is R11, R21, R31, R12, R22, R32, R13, R23, R33, Tx, Ty, Tz
 *        output is d_R11/d_v1, d_R11/d_v2, etc
 * @return true if successful, false if not
 */

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    // dR11
    jacobian[0] = 0;
    jacobian[1] = x[2];
    jacobian[2] = -x[1];
    jacobian[3] = 0;
    jacobian[4] = 0;
    jacobian[5] = 0;
    // dR21
    jacobian[6] = -x[2];
    jacobian[7] = 0;
    jacobian[8] = x[0];
    jacobian[9] = 0;
    jacobian[10] = 0;
    jacobian[11] = 0;
    // dR31
    jacobian[12] = x[1];
    jacobian[13] = -x[0];
    jacobian[14] = 0;
    jacobian[15] = 0;
    jacobian[16] = 0;
    jacobian[17] = 0;
    // dR12
    jacobian[18] = 0;
    jacobian[19] = x[5];
    jacobian[20] = -x[4];
    jacobian[21] = 0;
    jacobian[22] = 0;
    jacobian[23] = 0;
    // dR22
    jacobian[24] = -x[5];
    jacobian[25] = 0;
    jacobian[26] = x[3];
    jacobian[27] = 0;
    jacobian[28] = 0;
    jacobian[29] = 0;
    // dR32
    jacobian[30] = x[4];
    jacobian[31] = -x[3];
    jacobian[32] = 0;
    jacobian[33] = 0;
    jacobian[34] = 0;
    jacobian[35] = 0;
    // dR13
    jacobian[36] = 0;
    jacobian[37] = x[8];
    jacobian[38] = -x[7];
    jacobian[39] = 0;
    jacobian[40] = 0;
    jacobian[41] = 0;
    // dR23
    jacobian[42] = -x[8];
    jacobian[43] = 0;
    jacobian[44] = x[6];
    jacobian[45] = 0;
    jacobian[46] = 0;
    jacobian[47] = 0;
    // dR33
    jacobian[48] = x[7];
    jacobian[49] = -x[6];
    jacobian[50] = 0;
    jacobian[51] = 0;
    jacobian[52] = 0;
    jacobian[53] = 0;
    // dTx
    jacobian[54] = 0;
    jacobian[55] = x[11];
    jacobian[56] = -x[10];
    jacobian[57] = 1;
    jacobian[58] = 0;
    jacobian[59] = 0;
    // dTy
    jacobian[60] = -x[11];
    jacobian[61] = 0;
    jacobian[62] = x[9];
    jacobian[63] = 0;
    jacobian[64] = 1;
    jacobian[65] = 0;
    // dTz
    jacobian[66] = x[10];
    jacobian[67] = -x[9];
    jacobian[68] = 0;
    jacobian[69] = 0;
    jacobian[70] = 0;
    jacobian[71] = 1;

    return true;
}

}