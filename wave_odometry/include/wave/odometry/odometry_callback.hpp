/**
 * This class uses the ceres EvaluationCallback API to transform each feature point around the current trajectory
 * estimate. The jacobians for each transformed point are also calculated. There is an option to reuse jacobians
 * from the previous step if the state has not changed by much.
 */

#ifndef WAVE_ODOMETRY_CALLBACK_HPP
#define WAVE_ODOMETRY_CALLBACK_HPP

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include "wave/utils/math.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/transformer.hpp"

namespace wave {

struct OdometryCallback : ceres::EvaluationCallback {
    explicit OdometryCallback(std::vector<std::vector<FeatureTrack<12>>> *tracks,
                              std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_pts,
                              std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_ptsT,
                              std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> *traj,
                              Transformer *transformer,
                              std::vector<Vec12, Eigen::aligned_allocator<Vec12>> *params)
        : ceres::EvaluationCallback(),
          tracks(tracks),
          feat_pts(feat_pts),
          feat_ptsT(feat_ptsT),
          traj(traj),
          transformer(transformer),
          params(params) {}

    virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point);
    // indexed by feature type, then track index
    std::vector<std::vector<FeatureTrack<12>>> *tracks;
    // indexed by scan, then by feature type
    std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_pts, *feat_ptsT;

    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> *traj;

    Transformer *transformer;

    /// State variables
    std::vector<Vec12, Eigen::aligned_allocator<Vec12>> *params;

 private:
    bool old_jacobians = true;

    void evaluateJacobians();
};
}

#endif  // WAVE_ODOMETRY_CALLBACK_HPP
