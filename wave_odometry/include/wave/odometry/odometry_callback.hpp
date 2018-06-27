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
#include "wave/geometry/transformation.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/transformer.hpp"

namespace wave {

struct OdometryCallback : ceres::EvaluationCallback {
    explicit OdometryCallback(const std::vector<std::vector<FeatureTrack>> *tracks,
                              const std::vector<std::vector<Eigen::Tensor<float, 2>, Eigen::aligned_allocator<Eigen::Tensor<float, 2>>>> *feat_pts,
                              std::vector<std::vector<Eigen::Tensor<float, 2>, Eigen::aligned_allocator<Eigen::Tensor<float, 2>>>> *feat_ptsT,
                              const std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> *traj,
                              const std::vector<float> *traj_stamps,
                              Transformer *transformer)
        : ceres::EvaluationCallback(),
          tracks(tracks),
          feat_pts(feat_pts),
          feat_ptsT(feat_ptsT),
          traj(traj),
          transformer(transformer) { }

    virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point);
    // indexed by feature type, then track index
    const std::vector<std::vector<FeatureTrack>> *tracks;
    // indexed by scan, then by feature type
    const std::vector<std::vector<Eigen::Tensor<float, 2>, Eigen::aligned_allocator<Eigen::Tensor<float, 2>>>> *feat_pts;
    const std::vector<std::vector<Eigen::Tensor<float, 2>, Eigen::aligned_allocator<Eigen::Tensor<float, 2>>>> *interp_factors;
    std::vector<std::vector<Eigen::Tensor<float, 2>, Eigen::aligned_allocator<Eigen::Tensor<float, 2>>>> *feat_ptsT;

    ///State Variables, hooked to and updated by Ceres
    const std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> *traj;

    const std::vector<float> *traj_stamps;

    /// Cached intermediate variables for Jacobian calculation
    std::vector<Vec6, Eigen::aligned_allocator<Vec6>> pose_diff;
    std::vector<Mat6, Eigen::aligned_allocator<Mat6>> J_logmaps;

    Transformer *transformer;

 private:
    bool old_jacobians = true;

    void evaluateJacobians();
};
}

#endif  // WAVE_ODOMETRY_CALLBACK_HPP
