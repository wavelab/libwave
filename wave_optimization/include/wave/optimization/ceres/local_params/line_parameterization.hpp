#ifndef WAVE_LINE_PARAMETERIZATION_HPP
#define WAVE_LINE_PARAMETERIZATION_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include "wave/utils/math.hpp"

namespace wave {

class LineParameterization : public ceres::LocalParameterization {
 public:
    virtual ~LineParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 4; }
};

}

#endif //WAVE_LINE_PARAMETERIZATION_HPP
