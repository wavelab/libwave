/**
 * Trajectory prior for use with coupled state parameterization
 */

#ifndef WAVE_TRAJECTORY_PRIOR_COUPLED_HPP
#define WAVE_TRAJECTORY_PRIOR_COUPLED_HPP

#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

template<typename T_TYPE, int DIM>
class TrajectoryPriorCoupled : public ceres::SizedCostFunction<12, DIM> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    using Vec12 = Eigen::Matrix<double, 12, 1>;

    const int idx_k;
    const T_TYPE &inv_prior;
    const Vec6 &twist_prior;
    /// Set to be the square root of the inverse covariance
    const Mat12 weight_matrix;

 public:
    virtual ~TrajectoryPriorCoupled();

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    TrajectoryPriorCoupled(Mat12 weight_matrix, const T_TYPE &inv_prior, const Vec6 &twist_prior, const int &idx_k);
};

}

#include "wave/optimization/ceres/odom_gp_coupled_states/impl/trajectory_prior_coupled_impl.hpp"

#endif //WAVE_TRAJECTORY_PRIOR_COUPLED_HPP
