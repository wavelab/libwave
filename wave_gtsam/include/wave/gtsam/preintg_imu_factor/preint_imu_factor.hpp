#ifndef WAVE_PREINTG_IMU_FACTOR_HPP
#define WAVE_PREINTG_IMU_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/gtsam/pose_vel_bias.hpp"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

namespace wave {

/**
 * Binary factor connecting two PoseVelBias states, providing the effect of
 * gtsam::CombinedImuFactor (a 6-ary factor).
 */
class PreintegratedImuFactor
  : public gtsam::NoiseModelFactor2<PoseVelBias, PoseVelBias> {
 private:
    gtsam::PreintegratedCombinedMeasurements pim;
    using Base = NoiseModelFactor2<PoseVelBias, PoseVelBias>;

 public:
    PreintegratedImuFactor(gtsam::Key S1,
                           gtsam::Key S2,
                           const gtsam::PreintegratedCombinedMeasurements &pim);

    gtsam::Vector evaluateError(
      const PoseVelBias &state_i,
      const PoseVelBias &state_j,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;
};
}

#endif  // WAVE_PREINTG_IMU_FACTOR_HPP
