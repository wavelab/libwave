#ifndef WAVE_TRAJECTORY_PRIOR_HPP
#define WAVE_TRAJECTORY_PRIOR_HPP

#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"

/**
 * Implements a prior residual on a trajectory parameterized by a pose and
 * velocity
 */
namespace wave {

template<typename T_TYPE>
class TrajectoryPrior : public ceres::SizedCostFunction<12, 12, 6> {
 private:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    using Vec12 = Eigen::Matrix<double, 12, 1>;
    const T_TYPE &inv_prior;
    const Vec6 &twist_prior;
    /// Set to be the square root of the inverse covariance
    const Mat12 weight_matrix;

    /// Storage
    mutable Eigen::Matrix<double, 12, 12> J_T;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~TrajectoryPrior() {}
    TrajectoryPrior(Mat12 weight_matrix, const T_TYPE &inv_prior, const Vec6 &twist_prior)
        : inv_prior(inv_prior), twist_prior(twist_prior), weight_matrix(weight_matrix) {
        if (!inv_prior.storage.allFinite()) {
            throw std::invalid_argument("Inverse Prior not finite");
        }
        if (!twist_prior.allFinite()) {
            throw std::invalid_argument("Twist Prior not finite");
        }
        if (!weight_matrix.allFinite()) {
            throw std::invalid_argument("PoseVel Prior weight matrix not finite");
        }
        this->J_T.setZero();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Mat34> tk_map(parameters[0], 3, 4);
        Transformation<Eigen::Map<const Mat34>, true> transform(tk_map);

        Eigen::Map<const Vec6> twist_map(parameters[1]);
        auto diff = transform * this->inv_prior;
        Eigen::Map<Eigen::Matrix<double, 12, 1>> residual(residuals, 12, 1);
        residual.block<6, 1>(0, 0) = diff.logMap();
        residual.block<6, 1>(6, 0) = twist_map - this->twist_prior;
        if (jacobians) {
            if (jacobians[0]) {
                this->J_T.template block<12, 6>(0, 0) =
                  this->weight_matrix.template block<12, 6>(0, 0);
                Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(jacobians[0], 12, 12) = this->J_T;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>>(jacobians[1], 12, 6) =
                  this->weight_matrix.template block<12, 6>(0, 6);
            }
        }
        residual = this->weight_matrix * residual;

        return true;
    }
};
}

#endif  // WAVE_TRAJECTORY_PRIOR_HPP
