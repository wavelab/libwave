#include "wave/gtsam/gps_factor_with_bias.hpp"

namespace wave {

GPSFactorWBias::GPSFactorWBias(gtsam::Key LOCAL_S1,
                               gtsam::Key BiasKey,
                               gtsam::Pose3 T_local_s1,
                               const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2(model, LOCAL_S1, BiasKey), T_LOCAL_S1(T_local_s1) {}

gtsam::Vector GPSFactorWBias::evaluateError(
  const gtsam::Pose3 &T_LOCAL_S1,
  const gtsam::Point3 &B_Z,
  boost::optional<gtsam::Matrix &> J_T_LOCAL_S1,
  boost::optional<gtsam::Matrix &> J_B_Z) const {
    gtsam::Matrix J_compose_left, J_compose_right;
    gtsam::Matrix J_between_right;
    gtsam::Matrix J_logmap, J_bias;

    gtsam::Rot3 IdenRot;
    gtsam::Pose3 Lifted_Bias(IdenRot, B_Z);

    gtsam::Pose3 bias_T_LOCAL_S1 =
      Lifted_Bias.compose(T_LOCAL_S1, J_compose_left, J_compose_right);
    auto est_T_eye =
      this->T_LOCAL_S1.between(bias_T_LOCAL_S1, boost::none, J_between_right);
    auto retval = gtsam::Pose3::Logmap(est_T_eye, J_logmap);

    if (J_T_LOCAL_S1) {
        J_T_LOCAL_S1->resize(T_LOCAL_S1.dimension, T_LOCAL_S1.dimension);

        *J_T_LOCAL_S1 = J_logmap * J_between_right * J_compose_right;
    }
    if (J_B_Z) {
        J_B_Z->resize(6, 3);
        J_bias.resize(6, 3);
        J_bias.setZero();
        J_bias(3, 0) = 1;
        J_bias(4, 1) = 1;
        J_bias(5, 2) = 1;

        *J_B_Z = J_logmap * J_between_right * J_compose_left * J_bias;
    }

    return retval;
}
}