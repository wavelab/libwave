#ifndef WAVE_TRANSFORM_PRIOR_HPP
#define WAVE_TRANSFORM_PRIOR_HPP

#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"

/**
 * Implements a prior residual on a transform. Use will null SE3 parameterization only
 */
namespace wave {

class TransformPrior : public ceres::SizedCostFunction<6, 12> {
 private:
    /// Actually the inverse of the prior
    Transformation<Eigen::Matrix<double, 3, 4>> prior;
    /// Set to be the square root of the information matrix
    const Mat6 weight_matrix;
 public:
    virtual ~TransformPrior() {}
    TransformPrior(Mat6 weight, Transformation<Eigen::Matrix<double, 3, 4>> prior) : prior(prior), weight_matrix(weight) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 4>> tk_ptr(parameters[0], 3, 4);
        Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>> transform(tk_ptr);
        Eigen::Map<Vec6> residual(residuals);
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> del_e_del_T(jacobians[0]);
            del_e_del_T.block<6,6>(0,6).setZero();

            Mat6 J_boxminus;
            residual = transform.manifoldMinusAndJacobian(this->prior, &J_boxminus, nullptr);
            del_e_del_T.block<6,6>(0,0).noalias() = this->weight_matrix * J_boxminus;
        } else {
            transform.manifoldMinus(this->prior, residual);
        }
        residual = this->weight_matrix * residual;

        return true;
    }
};

}

#endif //WAVE_TRANSFORM_PRIOR_HPP
