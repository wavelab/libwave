#ifndef WAVE_GPS_FACTOR_WITH_BIAS_IMPL_HPP
#define WAVE_GPS_FACTOR_WITH_BIAS_IMPL_HPP

namespace wave {

template<class T>
gtsam::Vector GPSFactorWBiasEx<T>::evaluateError(const T &m, boost::optional<gtsam::Matrix &> H) const {
    gtsam::Vector6 retval;
    gtsam::Matrix J_compose_left, J_compose_right, J_between_right, J_logmap;

    gtsam::Rot3 IdenRot;
    gtsam::Pose3 Lifted_Bias(IdenRot, m.bias);
    gtsam::Pose3 bias_T_LOCAL_S1 =
            Lifted_Bias.compose(m.pose, J_compose_left, J_compose_right);
    auto est_T_eye =
            this->T_LOCAL_S1.between(bias_T_LOCAL_S1, boost::none, J_between_right);

    retval = gtsam::Pose3::Logmap(est_T_eye, J_logmap);

    if (H) {
        H->resize(6, gtsam::traits<T>::dimension);
        H->setZero();

        gtsam::Matrix J_lift;
        J_lift.resize(6, 3);
        J_lift.setZero();
        J_lift.block<3, 3>(3, 0).setIdentity();

        H->block<6, 6>(0, T::pose_offset).noalias() = J_logmap * J_between_right * J_compose_right;
        H->block<6, 3>(0, T::bias_offset).noalias() = J_logmap * J_between_right * J_compose_left * J_lift;
    }
    return retval;
}

}

#endif  // WAVE_GPS_FACTOR_WITH_BIAS_IMPL_HPP
