#include "wave/odometry/sensor_model.hpp"

namespace wave {

RangeSensor::RangeSensor(RangeSensorParams params) : param(params) {
    this->cos_elev.resize(0);
    this->sin_elev.resize(0);

    for (uint16_t i = 0; i < this->param.rings; i++) {
        this->cos_elev.emplace_back(cos(this->param.elevation_angles(i,0)));
        this->sin_elev.emplace_back(sin(this->param.elevation_angles(i,0)));
    }
}

void RangeSensor::getEuclideanCovariance(const double *point,
                                         const uint16_t &ring,
                                         Eigen::Matrix3f &covar_euclid) const {
    /// First need to convert euclidean coordinates to spherical
    float az = std::atan2(point[1], point[0]);
    float ca = std::cos(az);
    float sa = std::sin(az);
    float rng = std::sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);

    this->J(0,0) = this->cos_elev[ring] * ca;
    this->J(0,1) = -rng*this->sin_elev[ring] * ca;
    this->J(0,2) = -rng*this->cos_elev[ring] * sa;

    this->J(1,0) = this->cos_elev[ring] * sa;
    this->J(1,1) = -rng*this->sin_elev[ring] * sa;
    this->J(1,2) = rng*this->cos_elev[ring] * ca;

    this->J(2,0) = this->sin_elev[ring];
    this->J(2,1) = rng*this->cos_elev[ring];
    this->J(2,2) = 0;

    covar_euclid.noalias() = J * this->param.sigma_spherical * J.transpose();
}

}