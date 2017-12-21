#ifndef WAVE_OPTIMIZATION_CERES_INTERPOLATEDSE3_HPP
#define WAVE_OPTIMIZATION_CERES_INTERPOLATEDSE3_HPP

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

namespace wave {

// The transform is parameterized by an SE3 object and an interpolation variable
// By varying the interpolation variable between 0 and 1, the constant velocity path to the SE3
// object is traced out, so a particular combination of variable and SE3 object parameterizes another SE3 quantity

// The global size is 12, twelve variables from the SE3 object. The interpolation variable is pointed to.
// variable. The local size is 6 because it is assumed that the scale is known.

// Don't use this, one parameter block may only have one local parameterization

class InterpolatedSE3 : public ceres::LocalParameterization {
 private:
    const double * const alpha;
 public:
    InterpolatedSE3(const double &alph) : alpha(&alph) {}
    virtual ~InterpolatedSE3() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 12; }
    virtual int LocalSize() const { return 6; }
};

}

#endif  // WAVE_OPTIMIZATION_CERES_INTERPOLATEDSE3_HPP