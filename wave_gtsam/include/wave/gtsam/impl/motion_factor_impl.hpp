#ifndef WAVE_MOTION_FACTOR_IMPL_HPP
#define WAVE_MOTION_FACTOR_IMPL_HPP

#include "wave/gtsam/motion_factor.hpp"

namespace wave {
template <typename T>
MotionFactor::MotionFactor(gtsam::Key S1,
                           gtsam::Key S2,
                           double delta_t,
                           const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2<T, T>(model, S1, S2), delt(delta_t) {}

template <>
gtsam::Vector MotionFactor<wave::PoseVelBias>::evaluateError(
  const wave::PoseVelBias &m1,
  const wave::PoseVelBias &m2,
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

template <>
gtsam::Vector MotionFactor<wave::PoseVel>::evaluateError(
  const wave::PoseVel &m1,
  const wave::PoseVel &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    gtsam::Vector retval;

    if (H1) {
        H1->resize(12, 12);
        H1->setIdentity();

        H1->block<6, 6>(0, 6).noalias() = this->delt * gtsam::Matrix6::Identity();
    }

    if (H2) {
        H2->resize(12, 12);
        *H2 = -1 * Eigen::Matrix<double, 12, 12>::Identity();
    }

    retval.resize(12, 1);

    retval.block<6, 1>(0, 0).noalias() =
            m1.vel * this->delt - m1.pose.localCoordinates(m2.pose);
    retval.block<6, 1>(6, 0).noalias() = m1.vel - m2.vel;

    return retval;
}
}

#endif  // WAVE_MOTION_FACTOR_IMPL_HPP
