#ifndef WAVE_GTSAM_HELPERS_HPP
#define WAVE_GTSAM_HELPERS_HPP

#include <gtsam/geometry/Pose3.h>

#include "wave/utils/math.hpp"
#include "wave/vision/dataset/VoDataset.hpp"

namespace wave {

/** Convert pose to gtsam type */
inline gtsam::Pose3 gtsamPoseFromEigen(const Quaternion &q, const Vec3 &p) {
    return gtsam::Pose3{gtsam::Rot3{q}, gtsam::Point3{p}};
}

/** Construct a camera pose from a state with a robot body pose
 *
 * Includes transformation from robot body to camera frame (the camera's z axis
 * looks along the robot's x axis).
 */
inline gtsam::Pose3 gtsamPoseFromState(const VoInstant &state) {
    // Transform from robot to camera frame
    // This involves the rotation sequence: -90 deg about initial x axis,
    // 0, then -90 deg about initial z axis.
    const auto q_BC = Quaternion{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                                 Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

    const Quaternion q_GC = state.robot_q_GB * q_BC;
    const auto &G_p_GC = state.robot_G_p_GB;

    return gtsamPoseFromEigen(q_GC, G_p_GC);
}

/** Get the tranformation between two gtsam poses */
inline gtsam::Pose3 poseBetween(const gtsam::Pose3 &from,
                                const gtsam::Pose3 &to) {
    auto between = from.inverse() * to;

    // demonstrate the math
    auto res = from * between;
    assert(to.translation().isApprox(res.translation()));

    return between;
}

/** Get camera pose transformation given two robot states */
inline gtsam::Pose3 poseBetweenStates(const VoInstant &from,
                                      const VoInstant &to) {
    auto pose = gtsamPoseFromState(to);
    auto prev_pose = gtsamPoseFromState(from);

    return poseBetween(prev_pose, pose);
}

}  // namespace wave

#endif  // WAVE_GTSAM_HELPERS_HPP
