#ifndef WAVE_OPTIMIZATION_CERES_SE3PARAMERTERIZATION_HPP
#define WAVE_OPTIMIZATION_CERES_SE3PARAMERTERIZATION_HPP

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

namespace wave {

// The plus vector is added on the left, translation components first

class SE3Parameterization : public ceres::LocalParameterization {
 public:
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 12; }
    virtual int LocalSize() const { return 6; }
};

}

#endif  // WAVE_OPTIMIZATION_CERES_SE3PARAMERTERIZATION_HPP