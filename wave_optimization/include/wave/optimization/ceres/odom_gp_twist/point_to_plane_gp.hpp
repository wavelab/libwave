#ifndef WAVE_POINT_TO_PLANE_GP_TWIST_HPP
#define WAVE_POINT_TO_PLANE_GP_TWIST_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/geometry_og/transformation.hpp"

namespace wave_optimization {

struct SE3PointToPlaneGPObjects {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Preallocate memory for jacobian calculations to avoid it during residual evaluation
    // Jacobian of the residual wrt the transformed point
    Eigen::Matrix<double, 1, 3> Jr_P;

    wave::Vec3 T0_pt;

    mutable wave::Vec6 T_cur_twist;
    mutable wave::Transformation<wave::Mat34, true> T_current;
    // Interpolation factors
    Eigen::Matrix<double, 6, 12> hat;

    Eigen::Matrix<double, 6, 12> candle;

    mutable wave::Mat6 Jexp;

    // Jacobian of the transformed point wrt the interpolated transform
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Jacobian of residual wrt the interpolated transform
    mutable Eigen::Matrix<double, 1, 6> Jr_T;
};

class SE3PointToPlaneGP : public ceres::SizedCostFunction<1, 12, 12> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const double *const ptA;
    const double *const ptB;
    const double *const ptC;

    double cBA_BC[3];
    double inv_den;

    SE3PointToPlaneGPObjects &objects;

 public:
    double weight;

    virtual ~SE3PointToPlaneGP() {}
    SE3PointToPlaneGP(const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    SE3PointToPlaneGPObjects &objects,
                    const wave::Mat3 &covZ,
                    bool use_weighting);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const;
};
}

#endif //WAVE_POINT_TO_PLANE_GP_TWIST_HPP
