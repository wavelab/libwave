#ifndef WAVE_POSE_PRIOR_HPP
#define WAVE_POSE_PRIOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

/**
 * GTSAM prior for pose designed for custom states
 */

namespace wave {

template <class T>
class PosePrior : public gtsam::NoiseModelFactor1<T> {
 private:
    gtsam::Pose3 prior;

 public:
    PosePrior(gtsam::Key key,
              gtsam::Pose3 m,
              const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key),
          prior(m) {}

    gtsam::Vector evaluateError(
      const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};
}

#include "wave/gtsam/impl/pose_prior_impl.hpp"

#endif  // WAVE_POSE_PRIOR_HPP
