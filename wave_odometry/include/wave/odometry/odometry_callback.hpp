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
#include "wave/geometry/transformation.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/transformer.hpp"

namespace wave {

struct OdometryCallback : ceres::EvaluationCallback {
    explicit OdometryCallback(const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts,
                              Vec<VecE<Eigen::Tensor<float, 2>>> *feat_ptsT,
                              const VecE<PoseVel> *traj,
                              const Vec<float> *traj_stamps,
                              Transformer *transformer);

    virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point);

    /// Input data indexed by scan then feature type
    const Vec<VecE<Eigen::Tensor<float, 2>>> *feat_pts;

    /// Output data shared with residuals
    Vec<VecE<Eigen::Tensor<float, 2>>> *feat_ptsT;

    ///State Variables, hooked to and updated by Ceres
    const VecE<PoseVel> *traj;

    /// Cached intermediate variables for Jacobian calculation
    Vec<VecE<Eigen::Tensor<float, 2>>> interp_factors;
    VecE<Vec6> pose_diff;
    VecE<Mat6> J_logmaps;

    /// Stored jacobians for each point, indexed by scan, feature type, and then state
    /// Each element is a Nx3xK tensor, where n is the point index and k is the dimension of
    /// the state. Shared with residuals
    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> *ptT_jacobians;

    const Vec<float> *traj_stamps;
    Transformer *transformer;

 private:
    bool old_jacobians = true;

    void evaluateJacobians();
};
}

#endif  // WAVE_ODOMETRY_CALLBACK_HPP
