#ifndef WAVE_MOTION_FACTOR_IMPL_HPP
#define WAVE_MOTION_FACTOR_IMPL_HPP

#include "wave/gtsam/motion_factor.hpp"

namespace wave {
template <>
inline gtsam::Vector
MotionFactor<wave::PoseVelBias, wave::PoseVelBias>::evaluateError(
  const wave::PoseVelBias &m1,
  const wave::PoseVelBias &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    gtsam::Vector retval;
    gtsam::imuBias::ConstantBias imu_bias_error;

    if (H1) {
        H1->resize(21, 21);
        H1->setIdentity();

        H1->block<6, 6>(0, 6).noalias() =
          this->delt * gtsam::Matrix6::Identity();
    }

    if (H2) {
        H2->resize(21, 21);
        *H2 = -1 * Eigen::Matrix<double, 21, 21>::Identity();
    }

    retval.resize(21, 1);
    retval.block<6, 1>(0, 0).noalias() =
      m1.vel * this->delt - m1.pose.localCoordinates(m2.pose);
    retval.block<6, 1>(6, 0).noalias() = m1.vel - m2.vel;
    retval.block<3, 1>(12, 0).noalias() = m1.bias - m2.bias;
    imu_bias_error = m1.imu_bias - m2.imu_bias;
    retval.block<6, 1>(15, 0) = imu_bias_error.vector();

    return retval;
}

template <>
inline gtsam::Vector
MotionFactor<wave::PoseVelImuBias, wave::PoseVelImuBias>::evaluateError(
  const wave::PoseVelImuBias &m1,
  const wave::PoseVelImuBias &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    gtsam::Vector retval;
    gtsam::imuBias::ConstantBias imu_bias_error;

    if (H1) {
        H1->resize(18, 18);
        H1->setIdentity();

        H1->block<6, 6>(0, 6).noalias() =
          this->delt * gtsam::Matrix6::Identity();
    }

    if (H2) {
        H2->resize(18, 18);
        *H2 = -1 * Eigen::Matrix<double, 18, 18>::Identity();
    }

    retval.resize(18, 1);
    retval.block<6, 1>(0, 0).noalias() =
      m1.vel * this->delt - m1.pose.localCoordinates(m2.pose);
    retval.block<6, 1>(6, 0).noalias() = m1.vel - m2.vel;
    imu_bias_error = m1.imu_bias - m2.imu_bias;
    retval.block<6, 1>(12, 0) = imu_bias_error.vector();
    return retval;
}

template <>
inline gtsam::Vector MotionFactor<wave::PoseVel, wave::PoseVel>::evaluateError(
  const wave::PoseVel &m1,
  const wave::PoseVel &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    gtsam::Vector retval;

    if (H1) {
        H1->resize(12, 12);
        H1->setIdentity();

        H1->block<6, 6>(0, 6).noalias() =
          this->delt * gtsam::Matrix6::Identity();
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
}  // namespace wave

#endif  // WAVE_MOTION_FACTOR_IMPL_HPP
