#ifndef WAVE_POSE_PRIOR_IMPL_HPP
#define WAVE_POSE_PRIOR_IMPL_HPP

namespace wave {

template<class T>
gtsam::Vector PosePrior<T>::evaluateError(const T &m, boost::optional<gtsam::Matrix &> H) const {
    gtsam::Vector6 retval;
    gtsam::Matrix J_between_right, J_logmap;

    auto est_T_eye =
            this->prior.between(m.pose, boost::none, J_between_right);

    retval = gtsam::Pose3::Logmap(est_T_eye, J_logmap);

    if (H) {
        H->resize(6, gtsam::traits<T>::dimension);
        H->setZero();

        H->block<6, 6>(0, T::pose_offset).noalias() = J_logmap * J_between_right;
    }
    return retval;
}

}

#endif //WAVE_POSE_PRIOR_IMPL_HPP
