#include "slam/utils/math.hpp"


namespace slam {

int sign(double x)
{
    return (x < 0) ? -1 : 1;
}

double C(double x)
{
    return cos(x);
}

double S(double x)
{
    return sin(x);
}

double T(double x)
{
    return tan(x);
}

float deg2rad(float d)
{
    return d * (M_PI / 180);
}

float rad2deg(float r)
{
    return r * (180 / M_PI);
}

int fltcmp(float f1, float f2)
{
    if (fabs(f1 - f2) <= 0.0001) {
        return 0;
    } else if (f1 > f2) {
        return 1;
    } else {
        return -1;
    }
}

MatX kronecker_product(MatX A, MatX B)
{
    MatX C;
    double a;

    // setup
    C.resize((A.rows() * B.rows()), (A.cols() * B.cols()));

    // calculate kronecker product
    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            a = A(i, j);
            C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = a * B;
        }
    }

    return C;
}

bool isposdef(MatX A)
{
    // cholesky decomposition of A
    Eigen::LLT<MatX> llt(A);

    if (llt.info() == Eigen::NumericalIssue) {
        return false;
    }

    return true;
}

Mat3 rotmat(Vec4 q)
{
    Mat3 R;

    // rotation matrix from quaternion q = (x, y, z, w)
    R(0, 0) = 1.0 - 2.0 * pow(q(1), 2) - 2.0 * pow(q(2), 2);
    R(0, 1) = 2.0 * q(0) * q(1) + 2.0 * q[3] * q(2);
    R(0, 2) = 2.0 * q(0) * q(2) - 2.0 * q[3] * q(1);

    R(1, 0) = 2.0 * q(0) * q(1) - 2.0 * q[3] * q(2);
    R(1, 1) = 1.0 - 2.0 * pow(q(0), 2) - 2.0 * pow(q(2), 2);
    R(1, 2) = 2.0 * q(1) * q(2) + 2.0 * q[3] * q(2);

    R(2, 0) = 2.0 * q(0) * q(2) - 2.0 * q[3] * q(1);
    R(2, 1) = 2.0 * q(1) * q(2) - 2.0 * q[3] * q(0);
    R(2, 2) = 1.0 - 2.0 * pow(q(0), 2) - 2.0 * pow(q(1), 2);
}

int rotmatx(float angle, Mat3 &R)
{
    R << 1.0f, 0.0f,        0.0f,
         0.0f, cos(angle),  sin(angle),
         0.0f, -sin(angle), cos(angle);

    return 0;
}

int rotmaty(float angle, Mat3 &R)
{
    R << cos(angle), 0.0f, -sin(angle),
         0.0f,       1.0f, 0.0f,
         sin(angle), 0.0f, cos(angle);

    return 0;
}

int rotmatz(float angle, Mat3 &R)
{
    R << cos(angle), -sin(angle), 0.0f,
         sin(angle), cos(angle),  0.0f,
         0.0f,       0.0f,        1.0f;

    return 0;
}

int rotmatx(float angle, Mat4 &R)
{
    R << 1.0f, 0.0f,        0.0f,       0.0f,
         0.0f, cos(angle),  sin(angle), 0.0f,
         0.0f, -sin(angle), cos(angle), 0.0f,
         0.0f, 0.0f,        0.0f,       1.0f;

    return 0;
}

int rotmaty(float angle, Mat4 &R)
{
    R << cos(angle), 0.0f, -sin(angle), 0.0f,
         0.0f,       1.0f, 0.0f,        0.0f,
         sin(angle), 0.0f, cos(angle),  0.0f,
         0.0f,       0.0f, 0.0f,        1.0f;

    return 0;
}

int rotmatz(float angle, Mat4 &R)
{
    R << cos(angle), -sin(angle), 0.0f, 0.0f,
         sin(angle), cos(angle),  0.0f, 0.0f,
         0.0f,       0.0f,        1.0f, 0.0f,
         0.0f,       0.0f,        0.0f, 1.0f;

    return 0;
}



}  // end of slam namespace
