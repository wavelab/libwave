#ifndef WAVE_OPTIMIZATION_CERES_BA_HPP
#define WAVE_OPTIMIZATION_CERES_BA_HPP

#include <typeinfo>

#include <ceres/ceres.h>

#include "wave/utils/utils.hpp"
#include "wave/optimization/ceres/extensions.hpp"


namespace wave {
namespace ceres {

class BAResidual {
public:
  double fx;
  double fy;
  double cx;
  double cy;

  double x;
  double y;

  bool origin;

  BAResidual(void) {
    this->fx = 0.0;
    this->fy = 0.0;
    this->cx = 0.0;
    this->cy = 0.0;

    this->x = 0.0;
    this->y = 0.0;

    this->origin = false;
  }

  BAResidual(Mat3 K, Vec2 x, bool origin) {
    this->fx = K(0, 0);
    this->fy = K(1, 1);
    this->cx = K(0, 2);
    this->cy = K(1, 2);

    this->x = x(0);
    this->y = x(1);

    this->origin = origin;
  }

  template <typename T>
  T vectorMagnitude(const T *const v, const T length) {
    T m;

    for (T i = 0; i < length; i++) {
      m += pow(v[i], 2);
    }
    m = sqrt(m);

    return m;
  }

  template <typename T>
  bool operator()(const T *const q,
                  const T *const c,
                  const T *const x,
                  T *residual) const {
    Eigen::Matrix<T, 3, 3> K, R;
    Eigen::Matrix<T, 3, 1> C, X;
    Eigen::Matrix<T, 3, 1> x_est;
    Eigen::Matrix<T, 2, 1> x_est_pixel;

    // camera intrinsics matrix
    K(0, 0) = T(this->fx);
    K(0, 1) = T(0.0);
    K(0, 2) = T(this->cx);

    K(1, 0) = T(0.0);
    K(1, 1) = T(this->fy);
    K(1, 2) = T(this->cy);

    K(2, 0) = T(0.0);
    K(2, 1) = T(0.0);
    K(2, 2) = T(1.0);

    // rotation matrix from quaternion q = (x, y, z, w)
    R(0, 0) = T(1) - T(2) * pow(q[1], 2) - T(2) * pow(q[2], 2);
    R(0, 1) = T(2) * q[0] * q[1] + T(2) * q[3] * q[2];
    R(0, 2) = T(2) * q[0] * q[2] - T(2) * q[3] * q[1];

    R(1, 0) = T(2) * q[0] * q[1] - T(2) * q[3] * q[2];
    R(1, 1) = T(1) - T(2) * pow(q[0], 2) - T(2) * pow(q[2], 2);
    R(1, 2) = T(2) * q[1] * q[2] + T(2) * q[3] * q[2];

    R(2, 0) = T(2) * q[0] * q[2] - T(2) * q[3] * q[1];
    R(2, 1) = T(2) * q[1] * q[2] - T(2) * q[3] * q[0];
    R(2, 2) = T(1) - T(2) * pow(q[0], 2) - T(2) * pow(q[1], 2);

    // camera center
    C << c[0], c[1], c[2];

    // 3D point
    X << x[0], x[1], x[2];

    // calculate reprojection error
    if (origin) {
      x_est = K * X;  // for image 1
    } else {
      x_est = K * R * (X - C);  // for image 2 and beyond
    }

    // convert predicted 2d point in homogenous coordinates
    // back to image coordinates
    x_est_pixel << x_est(0) / x_est(2), x_est(1) / x_est(2);

    // calculate residual error
    residual[0] = abs(T(this->x) - x_est_pixel(0));
    residual[1] = abs(T(this->y) - x_est_pixel(1));

    return true;
  }
};

class BundleAdjustment {
public:
  bool configured;

  Mat3 K;
  MatX x1_pts;
  MatX x2_pts;

  double **q;
  double **c;
  double **x;

  BundleAdjustment(void);
  ~BundleAdjustment(void);
  int configure(Mat3 K, MatX x1_pts, MatX x2_pts);
  int solve(MatX pt3d);
};

}  // end of ceres namespace
}  // end of wave namespace
#endif
