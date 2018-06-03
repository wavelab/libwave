#include "wave/gtsam/preintg_imu_factor/preint_imu_factor.hpp"

namespace wave {

PreintegratedImuFactor::PreintegratedImuFactor(
  gtsam::Key S1,
  gtsam::Key S2,
  const gtsam::PreintegratedCombinedMeasurements &pim)
    : Base{gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
           S1,
           S2},
      pim{pim} {}

gtsam::Vector PreintegratedImuFactor::evaluateError(
  const PoseVelBias &state_i,
  const PoseVelBias &state_j,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
    // Split up the PoseVelBias combined states into pose, vel, and IMU bias.
    // (ignore gps bias)
    // Then use code adapted from gtsam::CombinedImuFactor.
    const auto &bias_i = state_i.imu_bias;
    const auto &bias_j = state_j.imu_bias;
    const auto &pose_i = state_i.pose;
    const auto &pose_j = state_j.pose;

    // Note we use only linear velocity here
    const auto &vel_i = state_i.vel.tail<3>();
    const auto &vel_j = state_j.vel.tail<3>();

    // Calculate error wrt bias evolution model (random walk)
    gtsam::Matrix6 Hbias_i, Hbias_j;
    gtsam::Vector6 fbias =
      gtsam::traits<gtsam::imuBias::ConstantBias>::Between(
        bias_j, bias_i, H2 ? &Hbias_j : 0, H1 ? &Hbias_i : 0)
        .vector();

    gtsam::Matrix96 D_r_pose_i, D_r_pose_j, D_r_bias_i;
    gtsam::Matrix93 D_r_vel_i, D_r_vel_j;

    // Calculate error wrt preintegrated measurements
    gtsam::Vector9 r_Rpv =
      this->pim.computeErrorAndJacobians(pose_i,
                                         vel_i,
                                         pose_j,
                                         vel_j,
                                         bias_i,
                                         H1 ? &D_r_pose_i : 0,
                                         H2 ? &D_r_vel_i : 0,
                                         H1 ? &D_r_pose_j : 0,
                                         H2 ? &D_r_vel_j : 0,
                                         H1 ? &D_r_bias_i : 0);

    if (H1) {
        H1->resize(15, 21);

        // Jacobian wrt pose (Pi)
        H1->block<9, 6>(0, PoseVelBias::pose_offset).noalias() = D_r_pose_i;
        // adding: [dBiasAcc/dPi ; dBiasOmega/dPi]
        H1->block<6, 6>(9, PoseVelBias::pose_offset).setZero();

        // Jacobian wrt linear velocity
        H1->block<9, 3>(0, PoseVelBias::vel_offset + 3).noalias() = D_r_vel_i;
        // Jacobian of bias wrt linear velocity is zero
        H1->block<6, 3>(9, PoseVelBias::vel_offset + 3).setZero();
        // Jacobian of all wrt angular velocity is zero
        H1->block<15, 3>(0, PoseVelBias::vel_offset).setZero();

        // Jacobian of all wrt gps bias is zero
        H1->block<15, 3>(0, PoseVelBias::bias_offset).setZero();

        // Jacobian wrt imu bias
        H1->block<9, 6>(0, PoseVelBias::imu_bias_offset) = D_r_bias_i;
        // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
        H1->block<6, 6>(9, PoseVelBias::imu_bias_offset) = Hbias_i;
    }
    if (H2) {
        H2->resize(15, 21);

        // Jacobian wrt pose (Pj)
        H2->block<9, 6>(0, PoseVelBias::pose_offset).noalias() = D_r_pose_j;
        // adding: [dBiasAcc/dPj ; dBiasOmega/dPj]
        H2->block<6, 6>(9, PoseVelBias::pose_offset).setZero();

        // Jacobian wrt linear velocity
        H2->block<9, 3>(0, PoseVelBias::vel_offset + 3).noalias() = D_r_vel_j;
        // Jacobian of bias wrt linear velocity is zero
        H2->block<6, 3>(9, PoseVelBias::vel_offset + 3).setZero();
        // Jacobian of all wrt angular velocity is zero
        H2->block<15, 3>(0, PoseVelBias::vel_offset).setZero();

        // Jacobian of all wrt gps bias is zero
        H2->block<15, 3>(0, PoseVelBias::bias_offset).setZero();

        // Jacobian wrt imu bias_j is zero
        H2->block<9, 6>(0, PoseVelBias::imu_bias_offset).setZero();
        // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
        H2->block<6, 6>(9, PoseVelBias::imu_bias_offset) = Hbias_j;
    }

    // Return overall error
    gtsam::Vector r(15);
    r << r_Rpv, fbias;  // vector of size 15
    return r;
}
}
