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

    ImplicitPlaneResidual(const FeatureTrack<state_dim> &track,
                          const std::vector<std::vector<Eigen::Map<Eigen::MatrixXf>>> *feat_points,
                          const std::vector<MatXf, Eigen::aligned_allocator<MatXf>> *avg_points) :
            track(track), feat_points(feat_points), avg_points(avg_points) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    const FeatureTrack<state_dim> &track;
    const std::vector<std::vector<Eigen::Map<MatXf>>> *feat_points;
    const std::vector<MatXf, Eigen::aligned_allocator<MatXf>> *avg_points;
};
}

#include "impl/implicit_plane_impl.hpp"

#endif //WAVE_IMPLICIT_PLANE_HPP
