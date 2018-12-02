/**
 * This class uses the ceres EvaluationCallback API to transform each feature point around the current trajectory
 * estimate. The jacobians for each transformed point are also calculated. There is an option to reuse jacobians
 * from the previous step if the state has not changed by much.
 */

#ifndef WAVE_ODOMETRY_CALLBACK_HPP
#define WAVE_ODOMETRY_CALLBACK_HPP

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include "wave/utils/utils.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/transformer.hpp"

namespace wave {

struct OdometryCallback : ceres::EvaluationCallback {
    explicit OdometryCallback(const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts,
                              const VecE<Eigen::Tensor<float, 2>> *prev_feat_pts,
                              const VecE<Eigen::Tensor<float, 2>> *cur_feat_pts,
                              Vec<VecE<MatX>> *feat_ptsT,
                              VecE<MatX> *prev_feat_ptsT,
                              VecE<MatX> *cur_feat_ptsT,
                              const VecE<PoseVel> *traj,
                              Vec<Vec<VecE<MatX>>> *ptT_jacobians,
                              Vec<Vec<float>> *jac_stamps,
                              const Vec<float> *traj_stamps,
                              const Vec<float> *scan_stamps,
                              Transformer *transformer);

    OdometryCallback() = delete;

    virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point);

    /// Input data indexed by scan then feature type
    const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts;
    const VecE<Eigen::Tensor<float, 2>> *prev_feat_pts, *cur_feat_pts;

    /// Output data shared with residuals
    Vec<VecE<MatX>> *feat_ptsT;
    VecE<MatX> *prev_feat_ptsT, *cur_feat_ptsT;

    /// State Variables, hooked to and updated by Ceres
    const VecE<PoseVel> *traj;

    /// Grid of jacobians to interpolate between. The first index is the gap between
    /// states. The next index is which state, order is pose 1, twist 1, pose 2, twist 2.
    /// Finally, the last index is however many grid points there are.
    Vec<Vec<VecE<MatX>>> *jacobians;
    Vec<Vec<float>> *jac_stamps;

    const Vec<float> *traj_stamps;

    /// Points are stored with time relative to the start of the scan
    const Vec<float> *scan_stamps;
    Transformer *transformer;

    const Vec<Vec<Vec<bool>>> *skip_point;

 private:
    /// Cached intermediate variables for Jacobian calculation
    VecE<T_TYPE> Pose_diff;
    VecE<Vec6> pose_diff;
    VecE<Mat6> J_logmaps;

    bool old_jacobians = true;

    void evaluateJacobians();
};
}

#endif  // WAVE_ODOMETRY_CALLBACK_HPP
