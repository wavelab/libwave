#ifndef WAVE_QUARTIC_LOSS_HPP
#define WAVE_QUARTIC_LOSS_HPP
#include <ceres/loss_function.h>

namespace wave {

class QuarticLoss : public ceres::LossFunction {
 public:
    explicit QuarticLoss() {}
    virtual void Evaluate(double ip, double* op) const;
};

}  // namespace wave

#endif //WAVE_QUARTIC_LOSS_HPP
