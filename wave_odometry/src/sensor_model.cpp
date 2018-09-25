#include "wave/odometry/sensor_model.hpp"

namespace wave {

RangeSensor::RangeSensor(RangeSensorParams params) : param(params) {
}

void RangeSensor::getEuclideanCovariance(const double *point,
                                         Mat3 &covar_euclid) const {
    static Mat3 J;
    /// First need to convert euclidean coordinates to spherical
    double xy = std::sqrt(point[0]*point[0] + point[1]*point[1]);
    double rng = std::sqrt(xy*xy + point[2]*point[2]);
    double az = std::atan2(point[1], point[0]);
    double elev = std::atan2(point[2], xy);
    // cos and sin of azimuth
    double ca = std::cos(az);
    double sa = std::sin(az);
    // cos and sin of elevation
    double ce = std::cos(elev);
    double se = std::sin(elev);

    J(0,0) = ce * ca;
    J(0,1) = -rng*se * ca;
    J(0,2) = -rng*ce * sa;

    J(1,0) = ce * sa;
    J(1,1) = -rng*se * sa;
    J(1,2) = rng*ce * ca;

    J(2,0) = se;
    J(2,1) = rng*ce;
    J(2,2) = 0;

    covar_euclid.noalias() = J * this->param.sigma_spherical * J.transpose();
}

}