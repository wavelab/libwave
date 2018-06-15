#include "wave/optimization/ceres/local_params/spherical_parameterization.hpp"

namespace wave {

bool SphericalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Vec3> p(x);
    Eigen::Map<const Vec2> dp(delta);
    Eigen::Map<Vec3> ppdp(x_plus_delta);

    // Cross product of unit z vector with p
    Vec3 axis;
    axis << -p(1), p(0), 0.0;

    double c = p(2);

    Mat3 skew;
    skew << 0, -axis(2), axis(1),
           axis(2), 0, -axis(0),
           -axis(1), axis(0), 0;

    Mat3 R = Mat3::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;

    Vec3 v = R.block<3, 2>(0,0) * dp;

    double a_mag = dp.norm();

    ppdp = std::cos(a_mag) * p + std::sin(a_mag) * (v / a_mag);

    return true;
}

bool SphericalParameterization::ComputeJacobian(const double *x, double *jacobian) const {

}

}

