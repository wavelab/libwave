#ifndef WAVE_GPS_FACTOR_WITH_BIAS_GENERAL_HPP
#define WAVE_GPS_FACTOR_WITH_BIAS_GENERAL_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace wave {

/**
 * This class is written for online GPS measurements in mind. There is an
 * explicitly
 * estimated bias quantity so that the GPS walk can be accounted for
 * The model is
 * r = logmap(T_LOCAL_S2 * (T_LOCAL_S1 * (T_S1_S2 + bias))^-1)
 *
 * It is assumed that T_LOCAL_S1 is a measurement, the rest are states
 */
template <class T>
class GPSFactorWithBiasGeneral : public gtsam::NoiseModelFactor1<T> {
 private:
    gtsam::Pose3 T_LOCAL_S1;

 public:
    GPSFactorWithBiasGeneral(gtsam::Key key,
                             gtsam::Pose3 m,
                             const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key),
          T_LOCAL_S1(m) {}

    gtsam::Vector evaluateError(
      const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};
}

#include "wave/gtsam/impl/gps_factor_with_bias_impl.hpp"

#endif  // WAVE_GPS_FACTOR_WITH_BIAS_GENERAL_HPP
