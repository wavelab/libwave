#include "wave/optimization/gtsam/motion_factor.hpp"

namespace wave {

MotionFactor::MotionFactor(gtsam::Key S1,
                           gtsam::Key S2,
                           double delta_t,
                           const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2<PoseVelBias, PoseVelBias>(model, S1, S2),
      delt(delta_t) {}

gtsam::Vector MotionFactor::evaluateError(
  const wave::PoseVelBias &m1,
  const wave::PoseVelBias &m2,
  boost::optional<gtsam::Matrix &> H1,
  boost::optional<gtsam::Matrix &> H2) const {
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
    gtsam::Vector retval;
    retval.resize(15, 1);
    retval.block<6, 1>(0, 0).noalias() =
      m1.vel * this->delt - m1.pose.localCoordinates(m2.pose);
    retval.block<6, 1>(6, 0).noalias() = m1.vel - m2.vel;
    retval.block<3, 1>(12, 0).noalias() = m1.bias - m2.bias;
    return retval;
}
}