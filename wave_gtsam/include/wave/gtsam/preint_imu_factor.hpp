#ifndef WAVE_PREINT_IMU_FACTOR_HPP
#define WAVE_PREINT_IMU_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/gtsam/pose_vel_bias.hpp"
#include "wave/gtsam/pose_vel_imubias.hpp"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

namespace wave {

/**
 * Binary factor connecting two combined pose/vel/bias states, providing the
 * effect of gtsam::CombinedImuFactor (a 6-ary factor).
 *
 * @tparam StateType either PoseVelBias or PoseVelImuBias
 */
template <typename StateType>
class PreintegratedImuFactor
  : public gtsam::NoiseModelFactor2<StateType, StateType> {
 private:
    gtsam::PreintegratedCombinedMeasurements pim;
    using Base = gtsam::NoiseModelFactor2<StateType, StateType>;

 public:
    PreintegratedImuFactor(gtsam::Key S1,
                           gtsam::Key S2,
                           const gtsam::PreintegratedCombinedMeasurements &pim)
        : Base{gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
               S1,
               S2},
          pim{pim} {}

    gtsam::Vector evaluateError(
      const StateType &state_i,
      const StateType &state_j,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;
};
}  // namespace wave

#endif  // WAVE_PREINT_IMU_FACTOR_HPP
