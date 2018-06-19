#ifndef WAVE_IMPLICIT_PLANE_HPP
#define WAVE_IMPLICIT_PLANE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/geometry/transformation.hpp"
#include "wave/odometry/feature_track.hpp"

namespace wave {

template <int cnt, int state_dim, int... num>
class ImplicitPlaneResidual : public ceres::SizedCostFunction<cnt, 3, num...> {
 public:
    virtual ~ImplicitPlaneResidual() {}

    ImplicitPlaneResidual(const FeatureTrack<state_dim> &pts) : pts(pts) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    const FeatureTrack<state_dim> &pts;
};
}

#include "impl/implicit_plane_impl.hpp"

#endif //WAVE_IMPLICIT_PLANE_HPP
