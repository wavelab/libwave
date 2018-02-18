#ifndef WAVE_CONSTANT_VELOCITY_COUPLED_HPP
#define WAVE_CONSTANT_VELOCITY_COUPLED_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

template<int DIM>
class ConstantVelocityPriorCoupled : public ceres::SizedCostFunction<6, DIM> {
 private:
    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 6, 6> weight;
    const double delta_t;
    const int idx_k;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPriorCoupled() {}
    ConstantVelocityPriorCoupled(const Eigen::Matrix<double, 6, 6> &weight, const double &delta_t, const int &idx_k)
        : weight(weight), delta_t(delta_t), idx_k(idx_k) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#include "wave/optimization/ceres/odom_gp_coupled_states/impl/constant_velocity_impl.hpp"

#endif  // WAVE_CONSTANT_VELOCITY_COUPLED_HPP
