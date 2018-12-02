#ifndef WAVE_CONSTANT_VELOCITY_HPP
#define WAVE_CONSTANT_VELOCITY_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave {

class ConstantVelocityPrior : public ceres::SizedCostFunction<12, 12, 12, 6, 6> {
 private:
    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 12, 12> weight;
    const double delta_t;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPrior() {}
    ConstantVelocityPrior(const Eigen::Matrix<double, 12, 12> &weight, const double &delta_t)
        : weight(weight), delta_t(delta_t) { }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_HPP
