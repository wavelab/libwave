#ifndef WAVE_PLANE_IMPL_HPP
#define WAVE_PLANE_IMPL_HPP

#include "wave/odometry/geometry/plane.hpp"

namespace wave {

template <typename Scalar, int... states>
bool PlaneResidual<Scalar, states...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<Eigen::Matrix<double, 1, 1>> error(residuals);
    Eigen::Map<const Vec6> plane(&parameters[0][0]);

    // Calculate error
    Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> Pt(this->pt);
    Vec3 diff = Pt.template cast<double>() - plane.block<3, 1>(3, 0);
    error = this->weight * (diff.transpose() * plane.block<3, 1>(0, 0));

    if (std::isnan(residuals[0])) {
        std::stringstream ss_msg;
        ss_msg << "Nan in plane residual. Point is " << Pt << "\n Plane is " << plane;
        throw std::runtime_error(ss_msg.str());
    }

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> plane_jac(jacobians[0]);

            plane_jac.template block<1, 3>(0, 0).noalias() = this->weight * diff.transpose();
            plane_jac.template block<1, 3>(0, 3).noalias() = this->weight * -plane.block<3, 1>(0, 0).transpose();
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
MatX PlaneResidual<Scalar, states...>::getDerivativeWpT(const Eigen::MatrixBase<Derived> &normal) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
    return normal.transpose();
}

}

#endif  // WAVE_PLANE_IMPL_HPP
