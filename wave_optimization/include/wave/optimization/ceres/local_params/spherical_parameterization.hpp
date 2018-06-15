/**
 * To optimize over the set of a unit 2-sphere
 */

#ifndef WAVE_SPHERICAL_PARAMETERIZATION_HPP
#define WAVE_SPHERICAL_PARAMETERIZATION_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include "wave/utils/math.hpp"

namespace wave {

class SphericalParameterization : public ceres::LocalParameterization {
 public:
    virtual ~SphericalParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 3; }
    virtual int LocalSize() const { return 2; }
};

}

#endif //WAVE_SPHERICAL_PARAMETERIZATION_HPP
