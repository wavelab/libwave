/**
 * This class uses the ceres EvaluationCallback API to transform each feature point around the current trajectory
 * estimate. The jacobians for each transformed point are also calculated. There is an option to reuse jacobians
 * from the previous step if the state has not changed by much.
 */

#ifndef WAVE_CERES_CALLBACK_HPP
#define WAVE_CERES_CALLBACK_HPP

#include "wave/utils/math.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/odometry_types.hpp"
#include <ceres/ceres.h>

namespace wave {

struct OdometryCallback : ceres::EvaluationCallback {
    explicit OdometryCallback(wave::Vec3 *state_point,
                              std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_points) :
            ceres::EvaluationCallback(), state_point(state_point), feat_points(feat_points) {}

    virtual void PrepareForEvaluation(bool, bool) {
        feat_points->at(0).at(0).block<3, 1>(0, 2) = state_point->cast<float>();
    }

    // indexed by feature type, then track index
    std::vector<std::vector<FeatureTrack<12>>> *tracks;
    // indexed by scan, then by feature type
    std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_pts, *feat_ptsT;

    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> *traj;
};

}

#endif //WAVE_CERES_CALLBACK_HPP
