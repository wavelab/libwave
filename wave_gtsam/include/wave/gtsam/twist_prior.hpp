#ifndef WAVE_TWIST_PRIOR_HPP
#define WAVE_TWIST_PRIOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace wave {

template <class T>
class TwistPrior : public gtsam::NoiseModelFactor1<T> {
 private:
    Eigen::Matrix<double, 6, 1> prior;

 public:
    TwistPrior(gtsam::Key key,
               Eigen::Matrix<double, 6, 1> m,
               const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key),
          prior(m) {}

    gtsam::Vector evaluateError(
      const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};
}

#include "wave/gtsam/impl/twist_prior_impl.hpp"

#endif  // WAVE_TWIST_PRIOR_HPP
