#ifndef WAVE_IMUBIAS_PRIOR_HPP
#define WAVE_IMUBIAS_PRIOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace wave {

template <class T>
class ImuBiasPrior : public gtsam::NoiseModelFactor1<T> {
 private:
    Eigen::Matrix<double, 6, 1> prior;

 public:
    BiasPrior(gtsam::Key key,
              Eigen::Matrix<double, 6, 1> m,
              const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key),
          prior(m) {}

    gtsam::Vector evaluateError(
      const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};
}

#include "wave/gtsam/impl/imubias_prior_impl.hpp"

#endif  // WAVE_IMUBIAS_PRIOR_HPP
