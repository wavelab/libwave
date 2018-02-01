/**
 * This is a version of the point to line residual designed to be used with
 * a GP model
 *
 * !!! WARNING !!! use only with "null" SE3 local parameterization
 */

#ifndef WAVE_POINT_TO_LINE_GP_HPP
#define WAVE_POINT_TO_LINE_GP_HPP

#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class SE3PointToLineGP : public ceres::SizedCostFunction<2, 12, 12, 6, 6> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;

    double diff[3];
    double bottom;

    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt an the transformed point
    Eigen::Matrix<double, 3, 3> Jres_P;

    mutable Transformation<Eigen::Matrix<double, 3, 4>, true> T_current;

    // Interpolation factors
    const Eigen::Matrix<double, 6, 12> hat;

    const Eigen::Matrix<double, 6, 12> candle;

    // Interpolation Jacobians
    mutable Eigen::Matrix<double, 6, 6> JT_Ti, JT_Tip1, JT_Wi, JT_Wip1;
    // Jacobian of the Transformed point wrt the transformation
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Complete Jacobian will null row
    mutable Eigen::Matrix<double, 3, 6> Jr_T;
    // Reduced Jacobian
//    mutable Eigen::Matrix<double, 2, 12> Jr_Tk;
//    mutable Eigen::Matrix<double, 2, 12> Jr_Tkp1;

    /**
     * This is used to rotate the residual into a frame where one of the unit vectors is
     * parallel with the line between A and B. This allows for the reduction of dimensionality
     * from 3 to 2 without much additional Jacobian complexity.
     */

    Eigen::Matrix3d rotation;


 public:
    Mat2 weight_matrix;

    virtual ~SE3PointToLineGP() {}

    SE3PointToLineGP(const double *const p,
                   const double *const pA,
                   const double *const pB,
                   const Eigen::Matrix<double, 6, 12> &hat,
                   const Eigen::Matrix<double, 6, 12> &candle,
                   const Mat3 &CovZ,
                   bool calculate_weight);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif //WAVE_POINT_TO_LINE_GP_HPP
