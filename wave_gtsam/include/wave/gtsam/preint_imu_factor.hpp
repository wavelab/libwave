#ifndef WAVE_PREINT_IMU_FACTOR_HPP
#define WAVE_PREINT_IMU_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/gtsam/pose_vel_bias.hpp"
#include "wave/gtsam/pose_vel.hpp"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

namespace wave {

/**
 * 4-ary factor connecting two combined pose/vel/bias states and two IMU biases.
 * Provides the effect of gtsam::CombinedImuFactor (a 6-ary factor).
 *
 * @tparam StateType either PoseVel or PoseVelBias
 */
template <typename StateType>
class PreintegratedImuFactor
  : public gtsam::NoiseModelFactor4<StateType,
                                    StateType,
                                    gtsam::imuBias::ConstantBias,
                                    gtsam::imuBias::ConstantBias> {
 private:
    gtsam::PreintegratedCombinedMeasurements pim;
    using Base = gtsam::NoiseModelFactor4<StateType,
                                          StateType,
                                          gtsam::imuBias::ConstantBias,
                                          gtsam::imuBias::ConstantBias>;

 public:
    PreintegratedImuFactor(gtsam::Key S1,
                           gtsam::Key S2,
                           gtsam::Key B1,
                           gtsam::Key B2,
                           const gtsam::PreintegratedCombinedMeasurements &pim)
        : Base{gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
               S1,
               S2,
               B1,
               B2},
          pim{pim} {}

    gtsam::Vector evaluateError(
      const StateType &state_i,
      const StateType &state_j,
      const gtsam::imuBias::ConstantBias &bias_i,
      const gtsam::imuBias::ConstantBias &bias_j,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none) const;
};
}  // namespace wave

#endif  // WAVE_PREINT_IMU_FACTOR_HPP
