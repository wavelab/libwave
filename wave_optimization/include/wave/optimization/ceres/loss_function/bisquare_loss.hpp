#ifndef WAVE_BISQUARE_LOSS_HPP
#define WAVE_BISQUARE_LOSS_HPP

#include <ceres/loss_function.h>

namespace wave {

class BisquareLoss : public ceres::LossFunction {
 public:
    explicit BisquareLoss(double k) : k2_(k*k) {}
    virtual void Evaluate(double ip, double* op) const;

 private:
    const double k2_;
};

}  // namespace wave

#endif //WAVE_BISQUARE_LOSS_HPP
