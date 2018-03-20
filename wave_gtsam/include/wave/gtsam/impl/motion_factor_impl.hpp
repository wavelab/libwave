#ifndef WAVE_MOTION_FACTOR_IMPL_HPP
#define WAVE_MOTION_FACTOR_IMPL_HPP

#include "wave/gtsam/motion_factor.hpp"

namespace wave {
template <typename T1, typename T2>
gtsam::Vector MotionFactor<T1, T2>::evaluateError(
  const T1 &m1,
  const T2 &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    gtsam::Vector retval;

    if (H1) {
        H1->resize(15, 15);
        H1->setIdentity();

        H1->block<6, 6>(0, 6).noalias() =
          this->delt * gtsam::Matrix6::Identity();
    }

    if (H2) {
        H2->resize(15, 15);
        *H2 = -1 * Eigen::Matrix<double, 15, 15>::Identity();
    }

    retval.resize(15, 1);
    retval.block<6, 1>(0, 0).noalias() =
      m1.vel * this->delt - m1.pose.localCoordinates(m2.pose);
    retval.block<6, 1>(6, 0).noalias() = m1.vel - m2.vel;
    retval.block<3, 1>(12, 0).noalias() = m1.bias - m2.bias;

    return retval;
}
}  // namespace wave

#endif  // WAVE_MOTION_FACTOR_IMPL_HPP
