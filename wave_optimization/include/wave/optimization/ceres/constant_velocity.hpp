#ifndef WAVE_CONSTANT_VELOCITY_HPP
#define WAVE_CONSTANT_VELOCITY_HPP

#include <memory>
#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class ConstantVelocityPrior : public ceres::SizedCostFunction<12, 12, 12, 6, 6> {
 private:
    /// Weight Matrix (square-root of information matrix)
    const Eigen::Matrix<double, 12, 12> weight;
    const double delta_t;

    ///Pre-allocated space for jacobian calculations
    mutable Eigen::Matrix<double, 12, 12> Jr_Ti, Jr_Tip1;
    mutable Eigen::Matrix<double, 12, 6> Jr_wip1;

    Eigen::Matrix<double, 12, 6> Jr_wi;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPrior() {}
    ConstantVelocityPrior(const Eigen::Matrix<double, 12, 12> &weight, const double &delta_t)
        : weight(weight), delta_t(delta_t) {
        this->Jr_Ti.block<12, 6>(0,6).setZero();
        this->Jr_Tip1.block<12, 6>(0,6).setZero();

        Eigen::Matrix<double, 12, 6> temp;
        temp.block<6,6>(0,0) = -this->delta_t * Mat6::Identity();
        temp.block<6,6>(6,0) = - Mat6::Identity();

        this->Jr_wi = this->weight * temp;

        this->Jr_wip1.setZero();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_HPP
