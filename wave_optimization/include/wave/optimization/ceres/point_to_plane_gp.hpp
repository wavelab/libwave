#ifndef WAVE_POINT_TO_PLANE_GP_HPP
#define WAVE_POINT_TO_PLANE_GP_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/geometry/transformation.hpp"

namespace wave {

class SE3PointToPlaneGP : public ceres::SizedCostFunction<1, 12, 12> {
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

    // Prior Transforms
    mutable Transformation T_current;
    const Transformation T_prior;
    const Transformation &T_k_inverse_prior;
    const Transformation &T_kp1_inverse_prior;

    // Jacobian of Interpolated transform wrt Tk
    const Eigen::Matrix<double, 6, 6> JT_Tk;
    // Jacobian of Interpolated transform wrt Tk+1
    const Eigen::Matrix<double, 6, 6> JT_Kp1;

    // Jacobian of the transformed point wrt the interpolated transform
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Jacobian of residual wrt the interpolated transform
    mutable Eigen::Matrix<double, 1, 6> Jr_T;

    // Complete Jacobians
    mutable Eigen::Matrix<double, 1, 12> Jr_Tk;
    mutable Eigen::Matrix<double, 1, 12> Jr_Tkp1;

 public:
    double weight;

    virtual ~SE3PointToPlaneGP() {}
    SE3PointToPlaneGP(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const Transformation &prior,
                    const Transformation &T_k_inverse_prior,
                    const Transformation &T_kp1_inverse_prior,
                    const Mat6 &JT_Tk,
                    const Mat6 &JT_Tkp1,
                    const Mat3 &covZ,
                    bool use_weighting);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const;
};
}

#endif //WAVE_POINT_TO_PLANE_GP_HPP
