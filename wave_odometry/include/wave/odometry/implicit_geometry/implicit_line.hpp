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
#include "wave/geometry/transformation.hpp"
#include "wave/odometry/feature_track.hpp"

namespace wave {

template <int cnt, int... num>
class ImplicitLineResidual : public ceres::SizedCostFunction<cnt, num...> {
 public:
    virtual ~ImplicitLineResidual() {}

    ImplicitLineResidual(const FeatureTrack &pts) : pts(pts) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    const FeatureTrack &pts;
};
}

#include "impl/implicit_line_impl.hpp"

#endif  // WAVE_IMPLICIT_LINE_HPP
