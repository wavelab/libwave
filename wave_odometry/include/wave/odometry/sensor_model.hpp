#ifndef WAVE_SENSOR_MODEL_HPP
#define WAVE_SENSOR_MODEL_HPP

#include <vector>
#include <Eigen/Core>

namespace wave {

/**
 * Parameters struct specifies the beam number and spacing
 */
struct RangeSensorParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    unsigned int rings;
    /// The elevation angles should be sorted in same order as feature rings
    std::vector<float> elevation_angles;

    /** Covariance of noise in spherical coordinates
    * Noise should be in linear, angular, angular units
     * on range, elevation, azimuth in that order
    */
    Eigen::Matrix3f sigma_spherical;
};

/**
 * Class to generate covariance matrices in Cartesian sensor frame given
 * a sensor variance supplied in Spherical Coordinates
 */
class RangeSensor {
 private:
    RangeSensor() = delete;
    const RangeSensorParams param;

    /// Number of elevation angles is small, can precalculate
    std::vector<float> sin_elev;
    std::vector<float> cos_elev;

    /// Hold some memory to avoid repeated allocation and deallocation
    mutable Eigen::Matrix3f J;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RangeSensor(RangeSensorParams params);

    void getEuclideanCovariance(const double *point, const uint16_t &ring,
                                Eigen::Matrix3f & covar_euclid) const;
};
}

#endif  // WAVE_SENSOR_MODEL_HPP
