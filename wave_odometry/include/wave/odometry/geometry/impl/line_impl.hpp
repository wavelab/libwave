#ifndef WAVE_LINE_IMPL_HPP
#define WAVE_LINE_IMPL_HPP

#include <wave/odometry/geometry/assign_jacobians.hpp>
#include "wave/odometry/geometry/line.hpp"

namespace wave {

namespace {

inline double getRotatedErrorAndJacobian(const Vec3 &error, Eigen::Matrix<double, 1, 3> *J) {
    Vec3 ed;
    double norm = error.norm();
    ed.noalias() = error / norm;

    if (std::isnan(ed.sum())) {
        ed.setZero();
        ed(0) = 1.0;
    }

    bool flipped = false;
    if (ed(0) < 0) {
        ed *= -1.0;
        flipped = true;
    }

    Eigen::Matrix<double, 1, 3> rotation;

    double scaling = 1.0 + ed(0);

    rotation(0) = 1.0 - (ed(1)*ed(1) + ed(2)*ed(2)) / scaling;
    rotation(1) = ed(1);
    rotation(2) = ed(2);

    if (J) {
        auto &del_re_del_e = *J;

        /// Technically speaking, R is dependent on the error, and therefore the state,
        /// however this adjustment in the Jacobian tends to be quite small so excluding it
        /// is a useful way to save time.
//        Mat3 del_ed_del_e;
//        if (norm < tolerance) {
//            del_ed_del_e.setZero();
//        } else {
//            del_ed_del_e = (1 / norm) * (Mat3::Identity() - ed * ed.transpose());
//        }
//        Mat3 del_R_del_ed;
//        del_R_del_ed.setZero();
//        del_R_del_ed(0,0) = (ed(1)*ed(1) + ed(2)*ed(2)) / (scaling*scaling);
//        del_R_del_ed(0,1) = -2.0 * ed(1) / scaling;
//        del_R_del_ed(0,2) = -2.0 * ed(2) / scaling;
//        del_R_del_ed(1,1) = 1.0;
//        del_R_del_ed(2,2) = 1.0;
//        Mat3 del_R_del_e;
//        if (flipped) {
//            del_R_del_e = -del_R_del_ed * del_ed_del_e;
//        } else {
//            del_R_del_e = del_R_del_ed * del_ed_del_e;
//        }

//        del_re_del_e(0) = rotation(0) + (del_R_del_e.col(0).transpose() * error)(0);
//        del_re_del_e(1) = rotation(1) + (del_R_del_e.col(1).transpose() * error)(0);
//        del_re_del_e(2) = rotation(2) + (del_R_del_e.col(2).transpose() * error)(0);

        del_re_del_e(0) = rotation(0);
        del_re_del_e(1) = rotation(1);
        del_re_del_e(2) = rotation(2);
    }

    return (rotation * error)(0); // rotated error
}

}

template <typename Scalar, int... states>
bool LineResidual<Scalar, states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Vec6> line(parameters[0]);

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
        Eigen::Matrix<double, 1, 3> del_re_del_e;
        residuals[0] = getRotatedErrorAndJacobian(error3, &del_re_del_e);
        Eigen::Matrix<double, 3, 3> del_e_del_diff;
        del_e_del_diff.noalias() = Mat3::Identity() - normal * normal.transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> del_re_del_line(jacobians[0]);

            Eigen::Matrix<double, 3, 6> del_e_del_line;
            // jacobian wrt normal
            del_e_del_line.block<3, 3>(0, 0).noalias() = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            del_e_del_line.block<3, 3>(0, 3).noalias() = -del_e_del_diff;

            del_re_del_line.noalias() = del_re_del_e * del_e_del_line;
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        del_ptT_T << 0, Pt(2), -Pt(1), 1, 0, 0, -Pt(2), 0, Pt(0), 0, 1, 0, Pt(1), -Pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 1, 6> del_re_del_T = del_re_del_e * del_e_del_diff * del_ptT_T;
        
        assignJacobian(jacobians + 1, del_re_del_T, this->jacsw1, this->jacsw2, this->w1, this->w2, 0, states...);
    } else {
        residuals[0] = getRotatedErrorAndJacobian(error3, nullptr);
    }
    return true;
}
}

#endif  // WAVE_LINE_IMPL_HPP
