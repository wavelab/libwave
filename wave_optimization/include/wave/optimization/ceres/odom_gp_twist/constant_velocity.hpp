#ifndef WAVE_CONSTANT_VELOCITY_TWIST_HPP
#define WAVE_CONSTANT_VELOCITY_TWIST_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave_optimization {

class ConstantVelocityPrior : public ceres::SizedCostFunction<12, 12, 12> {
 private:
    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 12, 12> weight;

    /// Error at the operating point
    const Eigen::Matrix<double, 12, 1> E_op;

    const double delta_t;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPrior() {}
    ConstantVelocityPrior(Eigen::Matrix<double, 12, 12> weight, Eigen::Matrix<double, 12, 1> E_op, double delta_t)
        : weight(std::move(weight)), E_op(std::move(E_op)), delta_t(delta_t) { }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_TWIST_HPP
