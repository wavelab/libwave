#include "wave/optimization/ceres/loss_function/bisquare_loss.hpp"

namespace wave {

void BisquareLoss::Evaluate(double s, double op[3]) const {
    // outlier region
    if (s > k2_) {
        op[0] = k2_/6.0;
        op[1] = 0;
        op[2] = 0;
    } else {
        // inlier region
        double frac = 1 - s/k2_;
        op[0] = (k2_/6) * (1 - frac*frac*frac);
        op[1] = frac*frac/2.0;
        op[2] = -frac;
    }
}

}