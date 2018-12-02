#ifndef WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP
#define WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP

#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave {

class SE3PointToLine : public ceres::SizedCostFunction<2, 12> {
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const scale;

    double diff[3];
    double bottom;

    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt the overparameterized transform
    Eigen::Matrix<double, 3, 12> Jres_T;

    // Jacobian of interpolated tranform wrt full transform
    mutable Mat6 J_int;

    // A couple lift Jacobians because Ceres
    mutable Eigen::Matrix<double, 12, 6> J_lift, J_lift_full;
    mutable Eigen::Matrix<double, 6, 12> J_lift_full_pinv;

    // Complete Jacobian will null row
    mutable Eigen::Matrix<double, 3, 12> Jr_T;
    // Reduced Jacobian
    mutable Eigen::Matrix<double, 2, 12> Jr_T_reduced;

    /**
     * This is used to rotate the residual into a frame where one of the unit vectors is
     * parallel with the line between A and B. This allows for the reduction of dimensionality
     * from 3 to 2 without much additional Jacobian complexity.
     */

    Eigen::Matrix3d rotation;


 public:
    Mat2 weight_matrix;

    virtual ~SE3PointToLine() {}
    SE3PointToLine(const double *const p,
                   const double *const pA,
                   const double *const pB,
                   const double *const scal,
                   const Mat3 &CovZ,
                   bool calculate_weight);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif  // WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP
