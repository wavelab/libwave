#ifndef WAVE_GPS_FACTOR_WITH_BIAS_HPP
#define WAVE_GPS_FACTOR_WITH_BIAS_HPP

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

class GPSFactorWBias
  : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
    gtsam::Pose3 T_LOCAL_S1;  // For example, a GPS measurement
 public:
    GPSFactorWBias(gtsam::Key LOCAL_S1,
                   gtsam::Key BiasKey,
                   gtsam::Pose3 T_local_s1,
                   const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
      const gtsam::Pose3 &T_LOCAL_S1,
      const gtsam::Point3 &B_Z,
      boost::optional<gtsam::Matrix &> J_T_LOCAL_S2 = boost::none,
      boost::optional<gtsam::Matrix &> J_B_Z = boost::none) const;
};
}

#endif  // WAVE_GPS_FACTOR_WITH_BIAS_HPP
