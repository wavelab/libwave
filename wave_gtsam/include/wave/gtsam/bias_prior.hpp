#ifndef WAVE_BIAS_PRIOR_HPP
#define WAVE_BIAS_PRIOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace wave {

template <class T>
class BiasPrior : public gtsam::NoiseModelFactor1<T> {
 private:
    Eigen::Matrix<double, 3, 1> prior;

 public:
    BiasPrior(gtsam::Key key,
              Eigen::Matrix<double, 3, 1> m,
              const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key),
          prior(m) {}

    gtsam::Vector evaluateError(
      const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};
}

#include "wave/gtsam/impl/bias_prior_impl.hpp"

#endif  // WAVE_BIAS_PRIOR_HPP
