#ifndef WAVE_HAND_EYE_HPP
#define WAVE_HAND_EYE_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace wave {

/**
 * This class is written with GPS - Sensor Calibration in mind. It is assumed
 * that
 * there is a common external reference frame.
 */

class HandEyeFactor
  : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Point3> {
    gtsam::Pose3 T_LOCAL_S1;  // For example, a GPS measurement
 public:
    HandEyeFactor(gtsam::Key O_S2,
                  gtsam::Key S1_S2,
                  gtsam::Key BiasKey,
                  gtsam::Pose3 T_local_s1,
                  const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
      const gtsam::Pose3 &T_LOCAL_S2,
      const gtsam::Pose3 &T_S1_S2,
      const gtsam::Point3 &B_Z,
      boost::optional<gtsam::Matrix &> J_T_LOCAL_S2 = boost::none,
      boost::optional<gtsam::Matrix &> J_T_S1_S2 = boost::none,
      boost::optional<gtsam::Matrix &> J_B_Z = boost::none) const;
};
}

#endif  // WAVE_HAND_EYE_HPP
