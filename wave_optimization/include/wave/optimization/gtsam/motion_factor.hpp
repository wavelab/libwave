#ifndef WAVE_MOTION_FACTOR_HPP
#define WAVE_MOTION_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/optimization/gtsam/pose_vel_bias.hpp"

// This factor propagates the pose_vel_bias state forward in time

namespace wave {

class MotionFactor : public gtsam::NoiseModelFactor2<PoseVelBias, PoseVelBias> {
 private:
    const double delt;

 public:
    MotionFactor(gtsam::Key S1,
                 gtsam::Key S2,
                 double delta_t,
                 const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
      const wave::PoseVelBias &m1,
      const wave::PoseVelBias &m2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;
};
}

#endif  // WAVE_MOTION_FACTOR_HPP
