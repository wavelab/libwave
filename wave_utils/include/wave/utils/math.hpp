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
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace wave {
/** @addtogroup utils
 *  @{ */

#ifndef EIGEN_TYPEDEF
#define EIGEN_TYPEDEF
typedef Eigen::Matrix<double, 1, 1> Vec1;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::Matrix<double, 5, 5> Mat5;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 12, 12> Mat12;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Matrix<double, 3, 4> Mat34;

typedef Eigen::Matrix<float, 1, 1> Vec1f;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix<float, 5, 1> Vec5f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::VectorXf VecXf;

typedef Eigen::Matrix2f Mat2f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Matrix<float, 5, 5> Mat5f;
typedef Eigen::Matrix<float, 6, 6> Mat6f;
typedef Eigen::Matrix<float, 12, 12> Mat12f;
typedef Eigen::MatrixXf MatXf;

typedef Eigen::Matrix<float, 3, 4> Mat34f;

typedef Eigen::Affine3d Affine3;

typedef Eigen::Quaterniond Quaternion;
#endif

/**
 * Eigen vector comparator
 */
struct VecComparator {
    bool operator()(const VecX &a, const VecX &b) const {
        return std::lexicographical_compare(
          a.data(), a.data() + a.size(), b.data(), b.data() + b.size());
    }
};

/**
 * Eigen matrix comparator
 */
struct MatComparator {
    bool operator()(const MatX &a, const MatX &b) const {
        return std::lexicographical_compare(
          a.data(), a.data() + a.size(), b.data(), b.data() + b.size());
    }
};

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

/** Wraps `euler_angle` to 180 degrees **/
double wrapTo180(double euler_angle);

/** Wraps `euler_angle` to 360 degrees **/
double wrapTo360(double euler_angle);

/** Convert euler angle to rotation matrix **/
int euler2rot(Vec3 euler, int euler_seq, Mat3 &R);

/** Convert euler angle to quaternion **/
int euler2quat(Vec3 euler, int euler_seq, Quaternion &q);

/** Convert quaternion to euler angles **/
int quat2euler(Quaternion q, int euler_seq, Vec3 &euler);

/** Convert quaternion to rotation matrix **/
int quat2rot(Quaternion q, Mat3 &R);

/** ENU to NWU coordinate system **/
void enu2nwu(const Vec3 &enu, Vec3 &nwu);

/** NED to ENU coordinate system **/
void ned2enu(const Vec3 &ned, Vec3 &enu);

/** NED to NWU coordinate system **/
void ned2nwu(const Quaternion &ned, Quaternion &enu);

/** NWU to ENU coordinate system **/
void nwu2enu(const Vec3 &nwu, Vec3 &enu);

/** NWU to NED coordinate system **/
void nwu2ned(const Quaternion &nwu, Quaternion &ned);

/** NWU to EDN coordinate system **/
void nwu2edn(const Vec3 &nwu, Vec3 &edn);

/** @} group utils */
}  // namespace wave

#endif  // WAVE_UTILS_MATH_HPP
