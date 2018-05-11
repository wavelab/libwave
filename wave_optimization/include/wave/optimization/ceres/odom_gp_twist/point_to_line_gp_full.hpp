/**
 * This is a version of the point to line residual designed to be used with
 * a GP model. In this version, each of the points are free to be transformed
 *
 * There are 3 or 4 states, depending on how far apart the points being interpolated across in time are
 *
 * Used with a twist parameterization of perturbed start and end transforms
 */

#ifndef WAVE_POINT_TO_LINE_GP_TWIST_FULL_HPP
#define WAVE_POINT_TO_LINE_GP_TWIST_FULL_HPP

#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry/transformation.hpp"

namespace wave_optimization {

struct SE3PointToLineGPFullObject {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt an the transformed point
    Eigen::Matrix<double, 3, 3> Jres_P;

    // Interpolation factors
    Eigen::Matrix<double, 6, 12> hat;

    Eigen::Matrix<double, 6, 12> candle;

    mutable wave::Mat6 Jexp;
    // Jacobian of the Transformed point wrt the transformation
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Complete Jacobian without null row
    mutable Eigen::Matrix<double, 2, 6> Jr_T;

    /**
     * This is used to rotate the residual into a frame where one of the basis vectors is
     * parallel with the line between A and B. This allows for the reduction of dimensionality
     * from 3 to 2 without much additional Jacobian complexity.
     */

    Eigen::Matrix3d rotation;
};

template <int... idx>
class SE3PointToLineGPFull : public ceres::SizedCostFunction<2, idx...> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const float *const pt;
    const float *const ptA;
    const float *const ptB;

    double diff[3];
    double bottom;

    SE3PointToLineGPFullObject &object;

 public:
    wave::Mat2 weight_matrix;

    virtual ~SE3PointToLineGP() {}

    SE3PointToLineGPFull(const float *const pt,
                         const float *const ptA,
                         const float *const ptB,
                         SE3PointToLineGPFullObject &object,
                         const wave::Mat3 &CovZ,
                         bool calculate_weight);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};

#include "wave/optimization/ceres/odom_gp_twist/impl/point_to_line_gp_full_impl.hpp"
}

#endif  // WAVE_POINT_TO_LINE_GP_TWIST_FULL_HPP
