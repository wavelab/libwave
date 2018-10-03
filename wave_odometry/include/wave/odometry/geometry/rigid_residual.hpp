/***
 * This is an implementation of a soft rigid body constraint. The landmarks it is defined between and
 * the weight adjust the strength of the constraint
 */

#ifndef WAVE_LANDMARK_DISPLACEMENT_HPP
#define WAVE_LANDMARK_DISPLACEMENT_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/utils/math.hpp"

namespace wave {

class RigidResidual : public ceres::SizedCostFunction<3, 6, 6> {
 public:
    virtual ~RigidResidual() {}

    RigidResidual(const double &weight, const Vec3& disp)
            : weight(weight), disp(disp) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
 private:
    const double weight;
    const Vec3 disp;
};

}

#endif //WAVE_LANDMARK_DISPLACEMENT_HPP
