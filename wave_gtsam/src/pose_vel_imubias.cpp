#include "wave/gtsam/pose_vel_imubias.hpp"

namespace wave {
PoseVelImuBias operator*(const PoseVelImuBias &m1, const PoseVelImuBias &m2) {
    wave::PoseVelImuBias retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    retval.imu_bias = m1.imu_bias + m2.imu_bias;
    return retval;
}
}
