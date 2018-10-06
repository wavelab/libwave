#ifndef WAVE_FIXED_LINE_IMPL_HPP
#define WAVE_FIXED_LINE_IMPL_HPP

#include <wave/odometry/geometry/assign_jacobians.hpp>
#include "wave/odometry/geometry/fixed_line.hpp"

namespace wave {

template<typename Scalar, int... states>
bool FixedLineResidual<Scalar, states...>::Evaluate(double const *const *parameters, double *residuals,
                                               double **jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, 3, 4>> T_OL(parameters[0]);
    Eigen::Map<Vec3> error(residuals);

    Vec6 lineT;
    lineT.block<3,1>(0,0).noalias() = T_OL.template block<3,3>(0,0) * this->line.template block<3,1>(0,0);
    lineT.block<3,1>(3,0).noalias() = T_OL.template block<3,3>(0,0) * this->line.template block<3,1>(3,0) + T_OL.template block<3,1>(0,3);

    const auto &normal = lineT.block<3, 1>(0, 0);
    const auto &avg = lineT.block<3, 1>(3, 0);

    Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> Pt(this->pt);

    Vec3 diff = Pt.template cast<double>() - avg;
    double dp = (normal.transpose() * diff)(0);
    error = diff - dp * normal;

    if (std::isnan(residuals[0])) {
        throw std::runtime_error("nan in line residual, probably due to incorrect point index");
    }

    if (jacobians) {
        Eigen::Matrix<double, 3, 3> del_e_del_diff;
        del_e_del_diff.noalias() = Mat3::Identity() - normal * normal.transpose();
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> del_e_del_T_OL(jacobians[0]);

            Eigen::Matrix<double, 3, 6> del_e_del_line;
            // jacobian wrt normal
            del_e_del_line.block<3, 3>(0, 0).noalias() = -normal * diff.transpose() - dp * Mat3::Identity();
            // jacobian wrt pt on line
            del_e_del_line.block<3, 3>(0, 3).noalias() = -del_e_del_diff;

            Eigen::Matrix<double, 6, 12> del_line_del_T_OL;
            del_line_del_T_OL.setZero();

            Transformation<>::skewSymmetric3(-normal, del_line_del_T_OL.block<3,3>(0,0));
            Transformation<>::skewSymmetric3(-avg, del_line_del_T_OL.block<3,3>(3,0));
            del_line_del_T_OL.block<3,3>(3,3).setIdentity();

            del_e_del_T_OL.noalias() = this->weight * del_e_del_line * del_line_del_T_OL;
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        Transformation<>::skewSymmetric3(-Pt, del_ptT_T.block<3,3>(0,0));
        del_ptT_T.block<3,3>(0,3).setIdentity();

        Eigen::Matrix<double, 3, 6> del_e_del_T = this->weight * del_e_del_diff * del_ptT_T;

        assignJacobian<states...>(jacobians + 1, del_e_del_T, this->jacsw1, this->jacsw2, this->w1, this->w2, 0);
    }
    return true;
}

template<typename Scalar, int... states>
template<typename Derived>
MatX FixedLineResidual<Scalar, states...>::getDerivativeWpT(const Eigen::MatrixBase<Derived> &normal) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3)
    Eigen::Matrix<double, 3, 3> del_e_del_diff;
    del_e_del_diff.noalias() = Mat3::Identity() - normal * normal.transpose();
    Vec3 error = Vec3::Zero();
    Vec2 re;
    Eigen::Matrix<double, 2, 3> del_re_del_e;
    getRotatedErrorAndJacobian(error, normal, re, &del_re_del_e);

    return del_re_del_e * del_e_del_diff;
}

}

#endif //WAVE_FIXED_LINE_IMPL_HPP
