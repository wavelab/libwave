#include "wave/gtsam/pose_vel_bias.hpp"

namespace wave {

PoseVelBias operator*(const PoseVelBias &m1, const PoseVelBias &m2) {
    wave::PoseVelBias retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    retval.bias = m1.bias + m2.bias;
    retval.imu_bias = m1.imu_bias + m2.imu_bias;
    return retval;
}
}
