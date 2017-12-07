#ifndef WAVE_POSE_PRIOR_IMPL_HPP
#define WAVE_POSE_PRIOR_IMPL_HPP

namespace wave {

template<class T>
gtsam::Vector TwistPrior<T>::evaluateError(const T &m, boost::optional<gtsam::Matrix &> H) const {
    gtsam::Vector6 retval;
    gtsam::Matrix J_between_right, J_logmap;

    auto est_T_eye =
            this->prior.between(m.vel, boost::none, J_between_right);

    retval = gtsam::traits<T.vel>::Logmap(est_T_eye, J_logmap);

    if (H) {
        H->resize(6, gtsam::traits<T>::dimension);
        H->setZero();

        H->block<6, 6>(0, T::vel_offset).noalias() = J_logmap * J_between_right;
    }
    return retval;
}

}

#endif //WAVE_POSE_PRIOR_IMPL_HPP
