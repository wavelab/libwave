#ifndef WAVE_CONSTANT_VELOCITY_RED_HPP
#define WAVE_CONSTANT_VELOCITY_RED_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave {

class ConstantVelocityPriorRed : public ceres::SizedCostFunction<6, 12, 12, 6> {
 private:
    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 6, 6> weight;
    const double delta_t;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPriorRed() {}
    ConstantVelocityPriorRed(const Eigen::Matrix<double, 6, 6> &weight, const double &delta_t)
        : weight(weight), delta_t(delta_t) { }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_RED_HPP
