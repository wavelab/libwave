#ifndef WAVE_LINE_IMPL_HPP
#define WAVE_LINE_IMPL_HPP

#include <wave/odometry/geometry/assign_jacobians.hpp>
#include "wave/odometry/geometry/line.hpp"

namespace wave {

namespace {

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

template<typename Scalar, int... states>
bool LineResidual<Scalar, states...>::Evaluate(double const *const *parameters, double *residuals,
                                               double **jacobians) const {
    Eigen::Map<const Vec6> line(parameters[0]);
    Eigen::Map<Vec2> error(residuals);

    auto normal = line.block<3, 1>(0, 0);
    auto avg = line.block<3, 1>(3, 0);

    Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> Pt(this->pt);

    Vec3 diff = Pt.template cast<double>() - avg;
    double dp = (normal.transpose() * diff)(0);
    Vec3 error3 = diff - dp * normal;

    if (std::isnan(residuals[0])) {
        throw std::runtime_error("nan in line residual, probably due to incorrect point index");
    }

    if (jacobians) {
        Eigen::Matrix<double, 2, 3> del_re_del_e;

        getRotatedErrorAndJacobian(error3, normal, error, &del_re_del_e);
        Eigen::Matrix<double, 3, 3> del_e_del_diff;
        del_e_del_diff.noalias() = Mat3::Identity() - normal * normal.transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> del_re_del_line(jacobians[0]);

            Eigen::Matrix<double, 3, 6> del_e_del_line;
            // jacobian wrt normal
            del_e_del_line.block<3, 3>(0, 0).noalias() = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            del_e_del_line.block<3, 3>(0, 3).noalias() = -del_e_del_diff;

            del_re_del_line.noalias() = del_re_del_e * del_e_del_line;
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        del_ptT_T << 0, Pt(2), -Pt(1), 1, 0, 0, -Pt(2), 0, Pt(0), 0, 1, 0, Pt(1), -Pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 2, 6> del_re_del_T = del_re_del_e * del_e_del_diff * del_ptT_T;

        assignJacobian(jacobians + 1, del_re_del_T, this->jacsw1, this->jacsw2, this->w1, this->w2, 0, states...);
    } else {
        getRotatedErrorAndJacobian(error3, normal, error, nullptr);
    }
    return true;
}
}

#endif  // WAVE_LINE_IMPL_HPP
