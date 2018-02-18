#ifndef WAVE_POINT_TO_PLANE_GP_COUPLED_HPP
#define WAVE_POINT_TO_PLANE_GP_COUPLED_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "wave/geometry/transformation.hpp"

namespace wave {

template<int DIM>
class SE3PointToPlaneGPCoupled : public ceres::SizedCostFunction<1, DIM> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const int idx_k;

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
    const Eigen::Matrix<double, 6, 12> hat;

    const Eigen::Matrix<double, 6, 12> candle;

    // Interpolation Jacobians
    mutable Eigen::Matrix<double, 6, 6> JT_Ti, JT_Tip1, JT_Wi, JT_Wip1;

    // Jacobian of the transformed point wrt the interpolated transform
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Jacobian of residual wrt the interpolated transform
    mutable Eigen::Matrix<double, 1, 6> Jr_T;

 public:
    double weight;

    virtual ~SE3PointToPlaneGPCoupled() = default;
    SE3PointToPlaneGPCoupled(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const Eigen::Matrix<double, 6, 12> &JT_Tk,
                    const Eigen::Matrix<double, 6, 12> &JT_Tkp1,
                     const int &idx_k,
                    const Mat3 &covZ,
                    bool use_weighting);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const;
};
}

#include "wave/optimization/ceres/odom_gp_coupled_states/impl/point_to_plane_gp_impl.hpp"

#endif //WAVE_POINT_TO_PLANE_GP_COUPLED_HPP
