/** @file
 * @ingroup utils
 *
 * Utility mathematical functions that do not fit anywhere else.
 *
 * The type definitions are shorthands for declaring `Eigen::Vector`,
 * `Eigen::Matrix` and `Eigen::Quaternion` objects. At current we assume all
 * slam code development will be using double precision floating points.
 */

#ifndef WAVE_UTILS_MATH_HPP
#define WAVE_UTILS_MATH_HPP

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace wave {
/** @addtogroup utils
 *  @{ */

#ifndef EIGEN_TYPEDEF
#define EIGEN_TYPEDEF
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Matrix<double, 5, 5> Mat5;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Affine3d Affine3;

typedef Eigen::Quaterniond Quaternion;
#endif

/** Generates random integer with a upper bound `ub` and lower bound `lb` using
 * a uniform random distribution. */
int randi(int ub, int lb);
/** Generates random float with a upper bound `ub` and lower bound `lb` using a
 * uniform random distribution. */
double randf(double ub, double lb);
/** Compares two floating point numbers `f1` and `f2` with an error threshold
 * defined by `threshold`.
 * @return
 * - `0`: If `f1` == `f2` or the difference is less then `threshold`
 * - `1`: If `f1` > `f2`
 * - `-1`: If `f1` < `f2`
 */
int fltcmp(double f1, double f2, double threshold = 0.0001);

/** @return the median of `v`. */
double median(std::vector<double> v);
/** Converts degrees to radians. */
double deg2rad(double d);
/** Converts radians to degrees. */
double rad2deg(double r);
/** Reshapes a vector `x` to matrix `y` of size `rows` and `cols` */
void vec2mat(std::vector<double> x, int rows, int cols, MatX &y);
/** Reshapes a matrix to a vector*/
void mat2vec(MatX A, std::vector<double> &x);
double wrapTo180(double euler_angle);
double wrapTo360(double euler_angle);
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);
int quat2rot(Quaternion q, Mat3 &R);

/** @} group utils */
}  // namespace wave

#endif  // WAVE_UTILS_MATH_HPP
