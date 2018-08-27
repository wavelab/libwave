/**
 * This residual penalizes the distance of a number of points to a line fit to those points
 *
 * The derivative of the points wrt the transform parameters is calculated using ceres EvaluationCallback API. This
 * abstracts the transformation interpolation technique out of the residual
 */

#ifndef WAVE_LINE_HPP
#define WAVE_LINE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/odometry/geometry/assign_jacobians.hpp"

namespace wave {

/// error is 3 dimensional, followed by 6 dimensional vector for line, and then all the states
template <int... states>
class LineResidual : public ceres::SizedCostFunction<3, 6, states...> {
 public:
    virtual ~LineResidual() {}

    LineResidual(const float *pt,
                 VecE<const MatX*> &jacsw1,
                 VecE<const MatX*> &jacsw2,
                 const float &w1,
                 const float &w2)
        : pt(pt),
          jacsw1(std::move(jacsw1)),
          jacsw2(std::move(jacsw2)),
          w1(w1),
          w2(w2) { }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    // updated by evaluation callback
    const float *pt;
    const VecE<const MatX*> jacsw1, jacsw2;

    const float w1, w2;
};
}

#include "wave/odometry/geometry/impl/line_impl.hpp"

#endif  // WAVE_LINE_HPP
