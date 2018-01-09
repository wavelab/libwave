#ifndef WAVE_TRANSFORM_PRIOR_HPP
#define WAVE_TRANSFORM_PRIOR_HPP

#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"

/**
 * Implements a prior residual on a transform
 */
namespace wave {

class TransformPrior : public ceres::SizedCostFunction<6, 12> {
 private:
    /// Actually the inverse of the prior
    Transformation prior;
    /// Set to be the square root of the information matrix
    const Mat6 weight_matrix;
 public:
    virtual ~TransformPrior() {}
    TransformPrior(Mat6 weight, Transformation Prior) : prior(Prior.inverse()), weight_matrix(weight) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 4>> map(parameters[0]);
        Transformation transform;
        transform.getInternalMatrix() = map;
        auto diff = transform * prior;
        Vec6 residual = diff.logMap();
        if (jacobians && jacobians[0]) {
            auto J_logmap = Transformation::SE3LeftJacobian(residual, 1e-4).inverse();
            Eigen::Matrix<double, 12, 6> J_lift;
            transform.J_lift(J_lift);
            Eigen::Matrix<double, 6, 12> J_lift_pinv = (J_lift.transpose() * J_lift).inverse() * J_lift.transpose();

            Eigen::Matrix<double, 6, 12> temp = this->weight_matrix * J_logmap * J_lift_pinv;

            Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>>(jacobians[0], 6, 12) = temp;
        }
        residual = this->weight_matrix * residual;
        Eigen::Map<Eigen::Matrix<double, 6, 1>>(residuals, 6, 1) = residual;

        return true;
    }
};

}

#endif //WAVE_TRANSFORM_PRIOR_HPP
