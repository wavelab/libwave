#ifndef WAVE_ROTATE_ERROR_HPP
#define WAVE_ROTATE_ERROR_HPP

#include "wave/utils/math.hpp"

namespace wave {

template<typename Derived>
inline void
getRotatedErrorAndJacobian(const Vec3 &error,
                           const Vec3 &normal,
                           Eigen::MatrixBase<Derived> &r_error,
                           Eigen::Matrix<double, 2, 3> *J) {

    // Cross product of unit z vector with p
    Vec2 axis;
    axis << normal(1), -normal(0); //, 0.0;

    double c = normal(2);

    Mat3 skew = Mat3::Zero();
    skew(0, 2) = axis(1);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Mat3 R = Mat3::Identity() + skew + (1.0 / (1.0 + c)) * skew * skew;

    Vec3 rotated_error = R * error;

    r_error = rotated_error.block<2,1>(0,0);

    if (J) {
        auto &del_re_del_e = *J;

        del_re_del_e = R.block<2,3>(0,0);
    }
}

}

#endif //WAVE_ROTATE_ERROR_HPP
