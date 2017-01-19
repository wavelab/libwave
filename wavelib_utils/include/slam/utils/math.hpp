#ifndef __SLAM_UTILS_MATH_HPP__
#define __SLAM_UTILS_MATH_HPP__

#include <Eigen/SVD>
#include <Eigen/Geometry>


namespace slam {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2f Vec2;
  typedef Eigen::Vector3f Vec3;
  typedef Eigen::Vector4f Vec4;
  typedef Eigen::VectorXf VecX;

  typedef Eigen::Matrix2f Mat2;
  typedef Eigen::Matrix3f Mat3;
  typedef Eigen::Matrix4f Mat4;
  typedef Eigen::MatrixXf MatX;
#endif

int sign(double x);
double C(double x);
double S(double x);
double T(double x);
float deg2rad(float d);
float rad2deg(float r);
int fltcmp(float f1, float f2);
MatX kronecker_product(MatX A, MatX B);
bool isposdef(MatX A);
Mat3 rotmat(Vec4 q);

} // end of slam namespace
#endif
