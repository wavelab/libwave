#include "wave/odometry/loss_functions.hpp"

namespace wave {

void BisquareLoss::Evaluate(double s, double op[3]) const {
    double r = std::sqrt(s);
    // outlier region
    if (r > k_) {
        op[0] = k2_;
        op[1] = 0;
        op[2] = 0;
    } else {
        // inlier region
        double frac = 1 - s/k2_;
        op[0] = k2_ * (1 - frac*frac*frac);
        op[1] = 6.0*r*frac*frac;
        op[2] = 6.0*frac*frac - 24 * (s/k2_) * frac;
    }
}

}
