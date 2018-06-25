#ifndef WAVE_PLANE_PARAMETERIZATION_HPP
#define WAVE_PLANE_PARAMETERIZATION_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include "wave/utils/math.hpp"

namespace wave {

class PlaneParameterization : public ceres::LocalParameterization {
 public:
    virtual ~PlaneParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 3; }
};

}

#endif //WAVE_PLANE_PARAMETERIZATION_HPP
