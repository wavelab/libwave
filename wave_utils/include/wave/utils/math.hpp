#ifndef WAVE_UTILS_MATH_HPP
#define WAVE_UTILS_MATH_HPP

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace wave {

#ifndef EIGEN_TYPEDEF
#define EIGEN_TYPEDEF
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix <double, 5, 1> Vec5;
typedef Eigen::Matrix <double, 6, 1> Vec6;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Matrix <double, 5, 5> Mat5;
typedef Eigen::Matrix <double, 6, 6> Mat6;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Affine3d Affine3;

typedef Eigen::Quaterniond Quaternion;
#endif

int randi(int ub, int lb);
double randf(double ub, double lb);
int fltcmp(double f1, double f2, double threshold = 0.0001);
double median(std::vector<double> v);
double deg2rad(double d);
double rad2deg(double r);
void vec2mat(std::vector<double> x, int rows, int cols, MatX &y);
void mat2vec(MatX A, std::vector<double> &x);
double wrapTo180(double euler_angle);
double wrapTo360(double euler_angle);
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);

}  // end of wave namespace
#endif
