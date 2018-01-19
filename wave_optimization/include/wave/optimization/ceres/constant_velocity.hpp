#ifndef WAVE_CONSTANT_VELOCITY_HPP
#define WAVE_CONSTANT_VELOCITY_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class ConstantVelocityPrior : public ceres::SizedCostFunction<12, 12, 12, 6, 6> {
 private:
    /// Transform prior
    const Transformation& inv_prev_transform_prior;
    const Transformation& inv_cur_transform_prior;
    /// Velocity prior
    const Vec6& prev_vel_prior;
    const Vec6& cur_vel_prior;

    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 12, 12>& weight;

    const Eigen::Matrix<double, 12, 12>& transition_matrix;

    ///Pre-allocated space for jacobian calculations
    mutable Eigen::Matrix<double, 12, 12> J12;
    mutable Eigen::Matrix<double, 12, 6> J6;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPrior() {}
    ConstantVelocityPrior(const Eigen::Matrix<double, 12, 12> &weight,
                          const Transformation& inv_prev_prior,
                          const Transformation& inv_cur_prior,
                          const Vec6& prev_vel_prior,
                          const Vec6& cur_vel_prior,
                          const Eigen::Matrix<double, 12, 12> &transition_matrix)
        : inv_prev_transform_prior(inv_prev_prior),
          inv_cur_transform_prior(inv_cur_prior),
          prev_vel_prior(prev_vel_prior),
          cur_vel_prior(cur_vel_prior),
          weight(weight),
          transition_matrix(transition_matrix) {
        J12.block<12, 6>(0,6).setZero();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_HPP
