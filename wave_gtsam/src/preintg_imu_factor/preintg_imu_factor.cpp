#include "wave/gtsam/preintg_imu_factor/preintg_imu_factor.hpp"

namespace wave {

PreintgIMUFactor::PreintgIMUFactor(
  gtsam::Key S1,
  gtsam::Key S2,
  gtsam::Key S3,
  const gtsam::PreintegratedImuMeasurements &pim)
    : Base(gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
           S1,
           S2,
           S3),
      _PIM_(pim) {}

gtsam::Vector PreintgIMUFactor::evaluateError(
  const wave::PoseVelBias &m1,
  const wave::PoseVelBias &m2,
  const gtsam::imuBias::ConstantBias &m3,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2,
  boost::optional<gtsam::Matrix &> H3) const {
    if (H1) {
        H1->resize(9, 21);
        H1->setZero();
    }
    if (H2) {
        H2->resize(9, 21);
        H2->setZero();
    }
    if (H3) {
        H3->resize(9, 6);
        H3->setZero();
    }
    gtsam::Vector retval, imuErrorVal;
    retval.resize(9, 1);

    gtsam::Matrix imu_H1, imu_H2, imu_H3, imu_H4, imu_H5;
    imu_H1.resize(9, 6);
    imu_H2.resize(9, 3);
    imu_H3.resize(9, 6);
    imu_H4.resize(9, 3);
    imu_H5.resize(9, 6);
    gtsam::Pose3 pose_i, pose_j;
    gtsam::Velocity3 vel_i, vel_j;

    pose_i = m1.pose;
    vel_i = m1.vel.block(3, 0, 3, 1);
    gtsam::NavState state_i(pose_i, vel_i);

    pose_j = m2.pose;
    vel_j = m2.vel.block(3, 0, 3, 1);
    gtsam::NavState state_j(pose_j, vel_j);

    retval = _PIM_.computeErrorAndJacobians(
      pose_i, vel_i, pose_j, vel_j, m3, imu_H1, imu_H2, imu_H3, imu_H4, imu_H5);

    if (H1) {
        H1->block<6, 6>(0, 0).noalias() = imu_H1.block<6, 6>(0, 0);
        H1->block<3, 6>(6, 0).noalias() = imu_H1.block<3, 6>(6, 0);
        H1->block<6, 3>(0, 9).noalias() = imu_H2.block<6, 3>(0, 0);
        H1->block<3, 3>(6, 9).noalias() = imu_H2.block<3, 3>(6, 0);
    }
    if (H2) {
        H2->block<6, 6>(0, 0).noalias() = imu_H3.block<6, 6>(0, 0);
        H2->block<3, 6>(6, 0).noalias() = imu_H3.block<3, 6>(6, 0);
        H2->block<6, 3>(0, 9).noalias() = imu_H4.block<6, 3>(0, 0);
        H2->block<3, 3>(6, 9).noalias() = imu_H4.block<3, 3>(6, 0);
    }
    if (H3) {
        H3->block<9, 6>(0, 0).noalias() = imu_H5.block<9, 6>(0, 0);
    }
    return retval;
}
}
