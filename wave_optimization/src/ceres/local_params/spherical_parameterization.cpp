#include "wave/optimization/ceres/local_params/spherical_parameterization.hpp"

namespace wave {

bool SphericalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Vec3> X(x);
    Eigen::Map<const Vec2> D(delta);
    Eigen::Map<Vec3> XpD(x_plus_delta);

    // Cross product of unit z vector with p
    Vec2 axis;
    axis << -X(1), X(0); //, 0.0;

    double c = X(2);

    Mat3 skew = Mat3::Zero();
    skew(0, 2) = axis(1);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Mat3 R = Mat3::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;

    Vec3 v = R.block<3, 2>(0, 0) * D;

    double a_mag = ceres::sqrt(D(0) * D(0) + D(1) * D(1));

    XpD = ceres::cos(a_mag) * X + ceres::sin(a_mag) * (v / a_mag);

    return true;
}

bool SphericalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<const Vec3> X(x);
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> J(jacobian);

    Vec2 axis;
    axis << -X(1), X(0); //, 0.0;

    double c = X(2);

    Mat3 skew = Mat3::Zero();
    skew(0, 2) = axis(1);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Mat3 R = Mat3::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;

    J = R.block<3, 2>(0,0);

    return true;
}

}