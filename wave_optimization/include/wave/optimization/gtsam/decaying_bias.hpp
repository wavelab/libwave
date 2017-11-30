#ifndef WAVE_DECAYING_BIAS_HPP
#define WAVE_DECAYING_BIAS_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

namespace wave {

/**
 * This residual models exponentially decaying translational bias
 * b2 = b1 * exp(-dt/Tau)
 */

class DecayingBias
  : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Point3> {
    double factor;

 public:
    /**
     * Follows standard GTSAM constructor style, non-standard arguments are detailed
     * @param B_1
     * @param B_2
     * @param dT: Input, the time different between bias states
     * @param T:  Input, the time decay constant
     * @param model
     */
    DecayingBias(gtsam::Key B_1,
                 gtsam::Key B_2,
                 double dT,
                 double T,
                 const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
      const gtsam::Point3 &B_1,
      const gtsam::Point3 &B_2,
      boost::optional<gtsam::Matrix &> J_B_1 = boost::none,
      boost::optional<gtsam::Matrix &> J_B_2 = boost::none) const;
};
}

#endif  // WAVE_DECAYING_BIAS_HPP
