#include "wave/odometry/geometry/rigid_residual.hpp"

namespace wave {

bool RigidResidual::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Vec6> L1(parameters[0]);
    Eigen::Map<const Vec6> L2(parameters[1]);
    Eigen::Map<Vec3> error(residuals);

    error = this->weight * (this->disp - (L1.block<3,1>(3,0) - L2.block<3,1>(3,0)));

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double,3,6,Eigen::RowMajor>> J_L1(jacobians[0]);
            J_L1.block<3,3>(0,0).setZero();
            J_L1.block<3,3>(0,3) = - this->weight * Mat3::Identity();
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double,3,6,Eigen::RowMajor>> J_L1(jacobians[1]);
            J_L1.block<3,3>(0,0).setZero();
            J_L1.block<3,3>(0,3) = this->weight * Mat3::Identity();
        }
    }
    return true;
}

}

