#ifndef WAVE_LOSS_FUNCTIONS_HPP
#define WAVE_LOSS_FUNCTIONS_HPP

#include <ceres/loss_function.h>

namespace wave {

class BisquareLoss : public ceres::LossFunction {
 public:
    explicit BisquareLoss(double k) : k_(k), k2_(k*k) {}
    virtual void Evaluate(double ip, double* op) const;

 private:
    const double k_;
    const double k2_;
};

class HuberBisquareLoss : public ceres::LossFunction {
 public:
    explicit HuberBisquareLoss(double k) : k_(k) {}
    virtual void Evaluate(double ip, double* op) const;

 private:
    const double k_;
};

}  // namespace wave

#endif //WAVE_LOSS_FUNCTIONS_HPP
