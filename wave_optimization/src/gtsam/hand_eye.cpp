#include "wave/optimization/gtsam/hand_eye.hpp"

namespace wave {

HandEyeFactor::HandEyeFactor(gtsam::Key O_S2,
                             gtsam::Key S1_S2,
                             gtsam::Key BiasKey,
                             gtsam::Pose3 T_local_s1,
                             const gtsam::SharedNoiseModel &model) :
        NoiseModelFactor3(model, O_S2, S1_S2, BiasKey), T_LOCAL_S1(T_local_s1) {}

gtsam::Vector HandEyeFactor::evaluateError(
  const gtsam::Pose3 &T_LOCAL_S2,
  const gtsam::Pose3 &T_S1_S2,
  const double &B_Z,
  boost::optional<gtsam::Matrix &> J_T_LOCAL_S2,
  boost::optional<gtsam::Matrix &> J_T_S1_S2,
  boost::optional<gtsam::Matrix &> J_B_Z) const {
    gtsam::Matrix J_compose_left, J_compose_right;
    gtsam::Matrix J_compose_left2, J_compose_right2;
    gtsam::Matrix J_between_left, J_between_right;
    gtsam::Matrix J_logmap, J_bias;

    // R1, t1 * R2, t2 = R1R2, R1t2 + t1

//    gtsam::Rot3 rot_T_LOCAL_S1(this->T_LOCAL_S1.rotation());
//    gtsam::Point3 t_T_LOCAL_S1(this->T_LOCAL_S1.translation());
//    t_T_LOCAL_S1.data()[2] += B_Z;

    gtsam::Point3 D_bias;
    D_bias.data()[0] = 0;
    D_bias.data()[1] = 0;
    D_bias.data()[2] = B_Z;
    gtsam::Rot3 IdenRot;
    gtsam::Pose3 Lifted_Bias(IdenRot, D_bias);

    gtsam::Pose3 bias_T_S1_S2 = Lifted_Bias.compose(T_S1_S2, J_compose_left2, J_compose_right2);

    auto meas_T_LOCAL_S2 =
            this->T_LOCAL_S1.compose(bias_T_S1_S2, boost::none, J_compose_right);
    auto est_T_eye =
      meas_T_LOCAL_S2.between(T_LOCAL_S2, J_between_left, J_between_right);
    auto retval = gtsam::Pose3::Logmap(est_T_eye, J_logmap);

    if (J_T_LOCAL_S2) {
        J_T_LOCAL_S2->resize(T_LOCAL_S2.dimension, T_LOCAL_S2.dimension);

        *J_T_LOCAL_S2 = J_logmap * J_between_right;
    }
    if (J_T_S1_S2) {
        J_T_S1_S2->resize(T_S1_S2.dimension, T_S1_S2.dimension);

        *J_T_S1_S2 = J_logmap * J_between_left * J_compose_right * J_compose_right2;
    }
    if (J_B_Z) {
        J_B_Z->resize(6, 1);
        J_bias.resize(6, 1);
        J_bias.setZero();
        J_bias(5) = 1;

        *J_B_Z = J_logmap * J_between_left * J_compose_right * J_compose_left2 * J_bias;
    }

    return retval;
}

}
