# wave/utils/math.hpp

This module contains utility mathematical functions that do not fit anywhere else.

## Type Definitions

    namespace wave {

    #ifndef EIGEN_TYPEDEF
    #define EIGEN_TYPEDEF
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

    }  // end of wave namespace

These type definitions are shorthands for declaring `Eigen::Vector`, `Eigen::Matrix` and `Eigen::Quaternion` objects. At current we assume all slam code development will be using double precision floating points.


## Functions

    int randi(int ub, int lb);
    double randf(double ub, double lb);

Generates random integer or float with a upper bound `ub` and lower bound `lb` using a uniform random distribution.

---

    int fltcmp(double f1, double f2, double threshold = 0.0001);

Compares two floating point numbers `f1` and `f2` with an error threshold defined by `threshold`.

Returns:

- `0`: If `f1` == `f2` or the difference is less then `threshold`
- `1`: If `f1` > `f2`
- `-1`: If `f1` < `f2`

---

    double median(std::vector<double> v);

Calculates and returns the median of double array `v`.

---

    double deg2rad(double d);
    double rad2deg(double r);

Convert degrees `d` to radians `r` and vice versa.

---

    void vec2mat(std::vector<double> x, int rows, int cols, MatX &y);
    void mat2vec(MatX A, std::vector<double> &x);

Convert between vector `x` to matrix `y` of size `rows` and `cols` and vice versa.

---

    double wrapTo180(double euler_angle);
    double wrapTo360(double euler_angle);

Wraps `euler_angle` to 180 or 360.
