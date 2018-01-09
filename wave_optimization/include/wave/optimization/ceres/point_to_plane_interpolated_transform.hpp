#ifndef WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
#define WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class SE3PointToPlane : public ceres::SizedCostFunction<1, 12> {
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const ptC;
    const double *const scale;
    double weight;

    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // This is the Jacobian of the cost function wrt the transformed point
    mutable Eigen::Matrix<double, 1, 3> Jr_P;
    // Jacobian of transformed point wrt the overparameterized transform
    Eigen::Matrix<double, 3, 12> JP_T;

    // Jacobian of interpolated tranform wrt full transform
    mutable Mat6 J_int;

    // A couple lift Jacobians because Ceres
    mutable Eigen::Matrix<double, 12, 6> J_lift, J_lift_full;
    mutable Eigen::Matrix<double, 6, 12> J_lift_full_pinv;

    // Complete Jacobian
    mutable Eigen::Matrix<double, 1, 12> Jr_T;

 public:
    virtual ~SE3PointToPlane() {}
    SE3PointToPlane(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const double *const scal,
                    double weighting)
            : pt(p), ptA(pA), ptB(pB), ptC(pC), scale(scal), weight(weighting) {
        this->JP_T << this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, 0, //
                0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1, 0, //
                0, 0, this->pt[0], 0, 0, this->pt[1], 0, 0, this->pt[2], 0, 0, 1;
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif //WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
