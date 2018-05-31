#ifndef WAVE_PREINTG_IMU_FACTOR_HPP
#define WAVE_PREINTG_IMU_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/gtsam/pose_vel_bias.hpp"
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
/**
 * This factor is designed for used with the PoseVelBias states. It implements
 * a simple constant velocity model with forward integration:
 *
 * S2 = S1 + delta_t * S1_velocity
 * S2_velocity = S2_velocity
 * S2_bias = S1_bias
 *
 * The error therefore has dimension 15 and is put together in the same order as
 * above
 *
 * The noise model should be used to weight each component of the error as
 * desired
 */

namespace wave {

class PreintgIMUFactor
  : public gtsam::NoiseModelFactor3<PoseVelBias,
                                    PoseVelBias,
                                    gtsam::imuBias::ConstantBias> {
 private:
    gtsam::PreintegratedImuMeasurements _PIM_;
    typedef NoiseModelFactor3<PoseVelBias,
                              PoseVelBias,
                              gtsam::imuBias::ConstantBias>
      Base;

 public:
    PreintgIMUFactor(gtsam::Key S1,
                     gtsam::Key S2,
                     gtsam::Key S3,
                     const gtsam::PreintegratedImuMeasurements &pim);

    gtsam::Vector evaluateError(
      const wave::PoseVelBias &m1,
      const wave::PoseVelBias &m2,
      const gtsam::imuBias::ConstantBias &m3,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none) const;
};
}

#endif  // WAVE_PREINTG_IMU_FACTOR_HPP
