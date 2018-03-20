#ifndef WAVE_MOTION_FACTOR_HPP
#define WAVE_MOTION_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "wave/gtsam/pose_vel.hpp"
#include "wave/gtsam/pose_vel_bias.hpp"

/**
 * This factor implements  a simple constant velocity model with forward
 * integration, and is designed to be used with PoseVel or PoseVelBias:
 *
 * S2 = S1 + delta_t * S1_velocity
 * S2_velocity = S2_velocity
 *
 * If being used with the PoseVelBias states:
 * S2_bias = S1_bias
 *
 * The error therefore has dimension 15 and is put together in the same order as
 * above
 *
 * The noise model should be used to weight each component of the error as
 * desired.
 */

namespace wave {

template <typename T>
class MotionFactor : public gtsam::NoiseModelFactor2<T, T> {
 private:
    const double delt;

 public:
    MotionFactor(gtsam::Key S1,
                 gtsam::Key S2,
                 double delta_t,
                 const gtsam::SharedNoiseModel &model);

    template <>
    gtsam::Vector evaluateError<wave::PoseVelBias>(
      const wave::PoseVelBias &m1,
      const wave::PoseVelBias &m2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;

    template <>
    gtsam::Vector evaluateError<wave::PoseVel>(
      const wave::PoseVel &m1,
      const wave::PoseVel &m2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;
};
}

#include "wave/gtsam/impl/motion_factor_impl.hpp"

#endif  // WAVE_MOTION_FACTOR_HPP
