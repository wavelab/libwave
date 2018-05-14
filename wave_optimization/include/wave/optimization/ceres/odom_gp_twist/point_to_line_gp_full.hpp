/**
 * This is a version of the point to line residual designed to be used with
 * a GP model. In this version, each of the points are free to be transformed
 *
 * There are 3 or 4 states, depending on how far apart the points being interpolated across in time are, and
 * there are 2 possible orderings for the 3 state case that must be distinguished between
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

    // Interpolation factors
    // Each point will have different factors
    std::vector<Eigen::Matrix<double, 1, 2>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 2>>> hat, candle;
    std::vector<wave::Mat6, Eigen::aligned_allocator<wave::Mat6>> Jlog;

    mutable Eigen::Matrix<double, 3, 3> Jres_pt, Jres_ptA, Jres_ptB;

    mutable wave::Mat6 Jexp, AdTMp1, AdTCp1, AdTMinv, AdTC;

    mutable wave::Vec6 twist;

    // Jacobian of the Transformed point wrt the transformation
    mutable Eigen::Matrix<double, 3, 6> JP_T;
    // Complete Jacobian without null row
    mutable Eigen::Matrix<double, 2, 6> Jr_T;

};

template <int... idx>
class SE3PointToLineGPFull : public ceres::SizedCostFunction<3, idx...> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
    const float *const pt;
    const float *const ptA;
    const float *const ptB;

    bool ahead;

    SE3PointToLineGPFullObject &object;

 public:
    wave::Mat3 weight_matrix;

    virtual ~SE3PointToLineGP() {}

    SE3PointToLineGPFull(const float *const pt, const float *const ptA, const float *const ptB,
                             SE3PointToLineGPFullObject &object, const wave::Mat3 &CovZ, bool calculate_weight, bool ahead = false);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};

#include "wave/optimization/ceres/odom_gp_twist/impl/point_to_line_gp_full_impl.hpp"
}

#endif  // WAVE_POINT_TO_LINE_GP_TWIST_FULL_HPP
