/**
 * This residual penalizes the distance of a number of points to a line fit to those points
 *
 * The derivative of the points wrt the transform parameters is calculated using ceres EvaluationCallback API. This
 * abstracts the transformation interpolation technique out of the residual
 */

#ifndef WAVE_IMPLICIT_LINE_HPP
#define WAVE_IMPLICIT_LINE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/odometry/implicit_geometry/assign_jacobians.hpp"
#include "wave/odometry/feature_track.hpp"

namespace wave {

/// error is 3 dimensional, followed by 6 dimensional vector for line, and then all the states
template <int... states>
class ImplicitLineResidual : public ceres::SizedCostFunction<3, 6, states...> {
 public:
    virtual ~ImplicitLineResidual() {}

    ImplicitLineResidual(const uint32_t &pt_id,
                         const uint32_t &feat_idx,
                         const FeatureTrack *track,
                         const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_points,
                         const Vec<VecE<MatXf>> *feat_points_T) :
            pt_id(pt_id), feat_idx(feat_idx), track(track), feat_points(feat_points), feat_points_T(feat_points_T) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    const uint32_t pt_id, feat_idx;
    const FeatureTrack *track;
    const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_points;
    const Vec<VecE<MatXf>> *feat_points_T;
};
}

#include "impl/implicit_line_impl.hpp"

#endif  // WAVE_IMPLICIT_LINE_HPP
