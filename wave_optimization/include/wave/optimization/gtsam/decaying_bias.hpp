#ifndef WAVE_DECAYING_BIAS_HPP
#define WAVE_DECAYING_BIAS_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace wave {

/**
 * This residual models exponentially decaying linear bias
 */

class DecayingBias : public gtsam::NoiseModelFactor2<double, double> {
    double factor;
 public:
    DecayingBias(gtsam::Key B_1, gtsam::Key B_2, double dT, double T, const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
            const double &B_1,
            const double &B_2,
            boost::optional<gtsam::Matrix &> J_B_1 = boost::none,
            boost::optional<gtsam::Matrix &> J_B_2 = boost::none) const;
};

}

#endif //WAVE_DECAYING_BIAS_HPP
