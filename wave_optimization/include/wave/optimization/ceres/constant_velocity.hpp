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

    const Eigen::Matrix<double, 12, 12>& transition_matrix;

    ///Pre-allocated space for jacobian calculations
    mutable Eigen::Matrix<double, 12, 12> J12;

    Eigen::Matrix<double, 12, 6> Jr_wi, Jr_wip1;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~ConstantVelocityPrior() {}
    ConstantVelocityPrior(const Eigen::Matrix<double, 12, 12> &weight, const double &delta_t)
        : weight(weight), delta_t(delta_t) {
        J12.block<12, 6>(0,6).setZero();
        Eigen::Matrix<double, 12, 6> temp;
        temp.block<6,6>(0,0) = -this->delta_t * Mat6::Identity();
        temp.block<6,6>(6,0) = - Mat6::Identity();
        this->Jr_wi = this->weight * temp;
        this->Jr_wip1.setZero();
        this->Jr_wip1.block<6,6>(6,0).setIdentity();

    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_CONSTANT_VELOCITY_HPP
