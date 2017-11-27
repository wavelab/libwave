#include "wave/optimization/gtsam/decaying_bias.hpp"

namespace wave {

DecayingBias::DecayingBias(gtsam::Key B_1,
                           gtsam::Key B_2,
                           double dT,
                           double T,
                           const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2(model, B_1, B_2) {
    this->factor = std::exp(-dT/T);
}

gtsam::Vector DecayingBias::evaluateError(const double &B_1, const double &B_2, boost::optional<gtsam::Matrix &> J_B_1,
                                          boost::optional<gtsam::Matrix &> J_B_2) const {

    if(J_B_1) {
        J_B_1->resize(1,1);
        (*J_B_1)(0,0) = -this->factor;
    }
    if(J_B_2) {
        J_B_2->resize(1,1);
        (*J_B_2)(0,0) = 1;
    }

    gtsam::Vector retval;
    retval.resize(1);
    retval(0) = B_2 - B_1 * (this->factor);
    return retval;
}

}