#ifndef __WAVE_UTILS_MATH_HPP__
#define __WAVE_UTILS_MATH_HPP__

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace wave {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Quaterniond Quaternion;
#endif

int randi(int ub, int lb);
double randf(double ub, double lb);
int fltcmp(double f1, double f2);
double median(std::vector<double> v);
double deg2rad(double d);
double rad2deg(double r);
void vec2mat(std::vector<double> x, int rows, int cols, MatX &y);
void mat2vec(MatX A, std::vector<double> &x);
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);
int quat2rot(Quaternion q, Mat3 &R);
void enu2nwu(Vec3 enu, Vec3 &nwu);
void cf2nwu(Vec3 cf, Vec3 &nwu);
void cf2enu(Vec3 cf, Vec3 &nwu);
void nwu2enu(Vec3 nwu, Vec3 &enu);
void ned2enu(Vec3 ned, Vec3 &enu);
void nwu2ned(Quaternion nwu, Quaternion &ned);
void ned2nwu(Quaternion ned, Quaternion &enu);
double wrapTo180(double euler_angle);
double wrapTo360(double euler_angle);

}  // end of wave namespace
#endif
