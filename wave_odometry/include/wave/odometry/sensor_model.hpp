#ifndef WAVE_SENSOR_MODEL_HPP
#define WAVE_SENSOR_MODEL_HPP

#include <vector>
#include <Eigen/Core>
#include "wave/utils/math.hpp"

namespace wave {

/**
 * Parameters struct specifies the beam number and spacing
 */
struct RangeSensorParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Covariance of noise in spherical coordinates
    * Noise should be in linear, angular, angular units
     * on range, elevation, azimuth in that order
    */
    Mat3 sigma_spherical = Mat3::Identity();
};

/**
 * Class to generate covariance matrices in Cartesian sensor frame given
 * a sensor variance supplied in Spherical Coordinates
 */
class RangeSensor {
 private:
    RangeSensor() = delete;
    const RangeSensorParams param;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RangeSensor(RangeSensorParams params);

    void getEuclideanCovariance(const double *point, Mat3& covar_euclid) const;
};
}

#endif  // WAVE_SENSOR_MODEL_HPP
