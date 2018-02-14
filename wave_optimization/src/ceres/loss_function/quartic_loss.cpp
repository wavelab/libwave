#include "wave/optimization/ceres/loss_function/quartic_loss.hpp"

namespace wave {

void QuarticLoss::Evaluate(double sq_norm, double *op) const {
    op[0] = 0.5 * sq_norm * sq_norm;
    op[1] = sq_norm;
    op[2] = 1;
}

}