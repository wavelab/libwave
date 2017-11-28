#ifndef WAVE_DECAYING_BIAS_HPP
#define WAVE_DECAYING_BIAS_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

namespace wave {

/**
 * This residual models exponentially decaying linear bias
 */

class DecayingBias : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Point3> {
    double factor;
 public:
    DecayingBias(gtsam::Key B_1, gtsam::Key B_2, double dT, double T, const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
            const gtsam::Point3 &B_1,
            const gtsam::Point3 &B_2,
            boost::optional<gtsam::Matrix &> J_B_1 = boost::none,
            boost::optional<gtsam::Matrix &> J_B_2 = boost::none) const;
};

}

#endif //WAVE_DECAYING_BIAS_HPP
