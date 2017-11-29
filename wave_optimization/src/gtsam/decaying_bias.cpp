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

gtsam::Vector DecayingBias::evaluateError(const gtsam::Point3 &B_1, const gtsam::Point3 &B_2, boost::optional<gtsam::Matrix &> J_B_1,
                                          boost::optional<gtsam::Matrix &> J_B_2) const {

    if(J_B_1) {
        J_B_1->resize(B_1.dimension,B_1.dimension);
        J_B_1->setIdentity();
        (*J_B_1) = -this->factor * (*J_B_1);
    }
    if(J_B_2) {
        J_B_2->resize(B_2.dimension,B_2.dimension);
        J_B_2->setIdentity();
    }

    gtsam::Vector retval;
    retval.resize(B_1.dimension);
    retval = B_2 - B_1 * (this->factor);
    return retval;
}

}