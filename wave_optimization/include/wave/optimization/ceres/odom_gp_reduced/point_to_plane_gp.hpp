#ifndef WAVE_POINT_TO_PLANE_GP_RED_HPP
#define WAVE_POINT_TO_PLANE_GP_RED_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/geometry_og/transformation.hpp"

namespace wave {

class SE3PointToPlaneGPRed : public ceres::SizedCostFunction<1, 12, 12, 6> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const ptC;

    double cBA_BC[3];
    double inv_den;

    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // Jacobian of the residual wrt the transformed point
    Eigen::Matrix<double, 1, 3> Jr_P;

    mutable Transformation<Eigen::Matrix<double, 3, 4>, true> T_current;
    // Interpolation factors
    const Mat12 hat;

    const Mat12 candle;

    // Interpolation Jacobians
    mutable Eigen::Matrix<double, 6, 6> JT_Ti, JT_Tip1, JT_Wi, JT_Wip1;

    // Jacobian of the transformed point wrt the interpolated transform
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Jacobian of residual wrt the interpolated transform
    mutable Eigen::Matrix<double, 1, 6> Jr_T;

 public:
    double weight;

    virtual ~SE3PointToPlaneGPRed() = default;
    SE3PointToPlaneGPRed(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const Mat12 &JT_Tk,
                    const Mat12 &JT_Tkp1,
                    const Mat3 &covZ,
                    bool use_weighting);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const;
};
}

#endif //WAVE_POINT_TO_PLANE_GP_RED_HPP
