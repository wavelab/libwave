/**
 * This is a version of the point to line residual designed to be used with
 * a GP model
 *
 * Used with a twist parameterization of perturbed start and end transforms
 */

#ifndef WAVE_POINT_TO_LINE_GP_TWIST_HPP
#define WAVE_POINT_TO_LINE_GP_TWIST_HPP

#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave_optimization {

struct SE3PointToLineGPObjects {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt an the transformed point
    Eigen::Matrix<double, 3, 3> Jres_P;

    wave::Vec3 T0_pt;

    mutable wave::Vec6 T_cur_twist;
    mutable wave::Mat34 T_current;

    // Interpolation factors
    Eigen::Matrix<double, 6, 12> hat;

    Eigen::Matrix<double, 6, 12> candle;

    mutable wave::Mat6 Jexp;
    // Jacobian of the Transformed point wrt the transformation
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Complete Jacobian without null row
    mutable Eigen::Matrix<double, 2, 6> Jr_T;

    /**
     * This is used to rotate the residual into a frame where one of the unit vectors is
     * parallel with the line between A and B. This allows for the reduction of dimensionality
     * from 3 to 2 without much additional Jacobian complexity.
     */

    Eigen::Matrix3d rotation;
};

class SE3PointToLineGP : public ceres::SizedCostFunction<2, 12, 12> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const double *const ptA;
    const double *const ptB;

    double diff[3];
    double bottom;

    SE3PointToLineGPObjects &objects;

 public:
    wave::Mat2 weight_matrix;

    virtual ~SE3PointToLineGP() {}

    SE3PointToLineGP(const double *const pA,
                   const double *const pB,
                   SE3PointToLineGPObjects &objects,
                   const wave::Mat3 &CovZ,
                   bool calculate_weight);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif //WAVE_POINT_TO_LINE_GP_TWIST_HPP
