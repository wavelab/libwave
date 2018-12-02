#ifndef WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
#define WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/geometry_og/transformation.hpp"

namespace wave {

class SE3PointToPlane : public ceres::SizedCostFunction<1, 12> {
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const ptC;
    const double *const scale;

    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt the overparameterized interpolated transform
    Eigen::Matrix<double, 1, 12> Jr_Ti;

    // Jacobian of interpolated tranform wrt full transform
    mutable Mat6 J_int;

    // A couple lift Jacobians because Ceres
    mutable Eigen::Matrix<double, 12, 6> J_lift, J_lift_full;
    mutable Eigen::Matrix<double, 6, 12> J_lift_full_pinv;

    // Complete Jacobian
    mutable Eigen::Matrix<double, 1, 12> Jr_T;

 public:
    double weight;

    virtual ~SE3PointToPlane() {}
    SE3PointToPlane(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const double *const scal,
                    const Mat3 &covZ,
                    bool use_weighting);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const;
};
}

#endif //WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
