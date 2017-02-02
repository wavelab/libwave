#ifndef __WAVE_UTILS_MATH_HPP__
#define __WAVE_UTILS_MATH_HPP__

#include <Eigen/SVD>
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
int randf(double ub, double lb);
int sign(double x);
float deg2rad(float d);
float rad2deg(float r);
int fltcmp(float f1, float f2);
MatX kronecker_product(MatX A, MatX B);
bool isposdef(MatX A);
Mat3 rotmat(Vec4 q);
void load_matrix(std::vector<double> x, int rows, int cols, MatX &y);
void load_matrix(MatX A, std::vector<double> &x);
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);
int quat2rot(Quaternion q, Mat3 &R);
void enu2nwu(Vec3 enu, Vec3 &nwu);
void cf2nwu(Vec3 cf, Vec3 &nwu);
void nwu2enu(Vec3 nwu, Vec3 &enu);
void cf2enu(Vec3 cf, Vec3 &nwu);

}  // end of wave namespace
#endif
