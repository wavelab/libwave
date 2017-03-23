#ifndef __WAVE_NUMERICAL_TEST_FUNCTORS_HPP__
#define __WAVE_NUMERICAL_TEST_FUNCTORS_HPP__

#include <iostream>
#include <functional>

#include "wave/kinematics/rotation.hpp"
#include "wave/utils/utils.hpp"
#include <kindr/Core>

namespace wave {

// The Functors are used for computing finite difference Jacobians
// of rotation expressions.

class RotateAndJacobianJpointFunctor {
public:
  wave::Rotation R;
  wave::Vec3 P;
  RotateAndJacobianJpointFunctor(wave::Rotation input_rotation) {
    this->R = input_rotation;
  }

  int operator()(wave::Vec3 input_point, wave::Vec3 &output_point) {
    output_point = R.rotate(input_point);
  }
};


class RotateAndJacobianJparamFunctor {
public:
  wave::Rotation R;
  wave::Vec3 P;
  RotateAndJacobianJparamFunctor(wave::Rotation input_rotation,
                                 wave::Vec3 input_point) {
    this->R = input_rotation;
    this->P = input_point;
  }

  int operator()(wave::Vec3 input_point, wave::Vec3 &output_point) {
    wave::Rotation Rp = this->R;
    Rp.manifoldPlus(input_point);
    output_point = Rp.rotate(P);
  }
};


// Implement the numerical differentiation.  Based on the Eigen::NumericalDiff
// Module.
template <typename MatrixType, typename FunctorType>
void numerical_jacobian(FunctorType F,
                        wave::Vec3 _x,
                        Eigen::MatrixBase<MatrixType> &jac) {
  using std::sqrt;
  double h;
  int n = _x.size();
  const double eps = sqrt(1e-10);
  wave::Vec3 val1, val2;
  wave::Vec3 x = _x;
  
  for (int j = 0; j < n; ++j) {
    h = eps * abs(x[j]);
    if (h == 0.) {
      h = eps;
    }

    x[j] += h;
    F.operator()(x, val2);
    x[j] -= 2 * h;
    F.operator()(x, val1);
    x[j] = _x[j];
    jac.col(j) = (val2 - val1) / (2 * h);
  }
}

}  // end namespace wave

#endif