#ifndef WAVE_FIXED_PLANE_IMPL_HPP
#define WAVE_FIXED_PLANE_IMPL_HPP

#include "wave/odometry/geometry/fixed_plane.hpp"

namespace wave {

template<typename Scalar, int... states>
bool FixedPlaneResidual<Scalar, states...>::Evaluate(double const *const *parameters, double *residuals,
                                                double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> error(residuals);
    Eigen::Map<const Eigen::Matrix<double, 3, 4>> T_OL(parameters[0]);

    Vec6 planeT;
    planeT.block<3,1>(0,0).noalias() = T_OL.template block<3,3>(0,0) * this->plane.template block<3,1>(0,0);
    planeT.block<3,1>(3,0).noalias() = T_OL.template block<3,3>(0,0) * this->plane.template block<3,1>(3,0) + T_OL.template block<3,1>(0,3);

    // Calculate error
    Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> Pt(this->pt);
    Vec3 diff = Pt.template cast<double>() - planeT.block<3, 1>(3, 0);
    error = this->weight * (diff.transpose() * planeT.block<3, 1>(0, 0));

    if (std::isnan(residuals[0])) {
        std::stringstream ss_msg;
        ss_msg << "Nan in plane residual. Point is " << Pt << "\n Plane is " << plane;
        throw std::runtime_error(ss_msg.str());
    }

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> del_e_del_T_OL(jacobians[0]);
            del_e_del_T_OL.block<1,6>(0,6).setZero();

            Eigen::Matrix<double, 1, 6> del_e_del_planeT;

            del_e_del_planeT.template block<1, 3>(0, 0).noalias() = this->weight * diff.transpose();
            del_e_del_planeT.template block<1, 3>(0, 3).noalias() = this->weight * -planeT.block<3, 1>(0, 0).transpose();

            Eigen::Matrix<double, 6, 6> del_planeT_del_T_OL;
            del_planeT_del_T_OL.setZero();

            Transformation<>::skewSymmetric3(-planeT.block<3, 1>(0, 0), del_planeT_del_T_OL.block<3,3>(0,0));
            Transformation<>::skewSymmetric3(-planeT.block<3, 1>(3, 0), del_planeT_del_T_OL.block<3,3>(3,0));
            del_planeT_del_T_OL.block<3,3>(3,3).setIdentity();

            del_e_del_T_OL.block<1,6>(0,0).noalias() = del_e_del_planeT * del_planeT_del_T_OL;
        }
        Eigen::Matrix<double, 3, 6> del_ptT_T;
        del_ptT_T << 0, Pt(2), -Pt(1), 1, 0, 0, -Pt(2), 0, Pt(0), 0, 1, 0, Pt(1), -Pt(0), 0, 0, 0, 1;

        Eigen::Matrix<double, 1, 6> del_e_del_T;
        del_e_del_T.noalias() = this->weight * plane.block<3, 1>(0, 0).transpose() * del_ptT_T;

        assignJacobian<states...>(jacobians + 1, del_e_del_T, this->jacsw1, this->jacsw2, this->w1, this->w2, 0);
    }

    return true;
}

template<typename Scalar, int... states>
template<typename Derived>
MatX FixedPlaneResidual<Scalar, states...>::getDerivativeWpT(const Eigen::MatrixBase<Derived> &normal) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
    return normal.transpose();
}

}

#endif //WAVE_FIXED_PLANE_IMPL_HPP
