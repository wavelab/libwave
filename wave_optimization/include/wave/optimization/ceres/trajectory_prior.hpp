#ifndef WAVE_TRAJECTORY_PRIOR_HPP
#define WAVE_TRAJECTORY_PRIOR_HPP

#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

/**
 * Implements a prior residual on a trajectory parameterized by a pose and
 * velocity
 */
namespace wave {

class TrajectoryPrior : public ceres::SizedCostFunction<12, 12, 6> {
 private:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    using Vec12 = Eigen::Matrix<double, 12, 1>;
    Transformation inv_prior;
    Vec6 twist_prior;
    /// Set to be the square root of the inverse covariance
    const Mat12 weight_matrix;

    /// Matrix to get around Ceres local parameterization stupidity
    Eigen::Matrix<double, 6, 12> dialate_mat;

 public:
    virtual ~TrajectoryPrior() {}
    TrajectoryPrior(Mat12 weight_matrix, Transformation inv_prior, Vec6 twist_prior)
        : inv_prior(inv_prior), twist_prior(twist_prior), weight_matrix(weight_matrix) {
        dialate_mat.block<6, 6>(0, 0).setIdentity();
        dialate_mat.block<6, 6>(0, 6).setZero();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 4>> map(parameters[0]);
        Eigen::Map<const Vec6> twist_map(parameters[1]);
        Transformation transform;
        transform.getInternalMatrix() = map;
        auto diff = transform * this->inv_prior;
        Vec12 residual;
        residual.block<6, 1>(0, 0) = diff.logMap();
        residual.block<6, 1>(6, 0) = twist_map - this->twist_prior;
        if (jacobians) {
            Mat12 J_full = Mat12::Identity();
            J_full.block<6, 6>(0, 0) = Transformation::SE3LeftJacobian(residual.block<6,1>(0,0), 1e-4).inverse();
            J_full = this->weight_matrix * J_full;

            if (jacobians[0]) {
                Eigen::Matrix<double, 12, 12> temp = J_full.block<12, 6>(0, 0) * this->dialate_mat;
                Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[0], 12, 12) = temp;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[1], 12, 6) =
                  J_full.block<12, 6>(0, 6);
            }
        }
        residual = this->weight_matrix * residual;
        Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals, 12, 1) = residual;

        return true;
    }
};
}

#endif  // WAVE_TRAJECTORY_PRIOR_HPP
