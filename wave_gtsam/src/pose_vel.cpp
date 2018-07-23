#include "wave/gtsam/pose_vel.hpp"

namespace wave {
PoseVel operator*(const PoseVel &m1, const PoseVel &m2) {
    wave::PoseVel retval;
    retval.pose = m1.pose * m2.pose;
    retval.vel = m1.vel + m2.vel;
    return retval;
}
}
