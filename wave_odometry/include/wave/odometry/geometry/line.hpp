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
#include "wave/odometry/geometry/rotate_error.hpp"

namespace wave {

/// error is 3 dimensional, followed by states: 6 dimensional vector for line, and then all the trajectory states
template <typename Scalar, int... states>
class LineResidual : public ceres::SizedCostFunction<3, 6, states...> {
 public:
    virtual ~LineResidual() {}

    LineResidual(const Scalar *pt,
                 VecE<const MatX*> &jacsw1,
                 VecE<const MatX*> &jacsw2,
                 const Scalar &w1,
                 const Scalar &w2)
        : pt(pt),
          jacsw1(std::move(jacsw1)),
          jacsw2(std::move(jacsw2)),
          w1(w1),
          w2(w2) { }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    template<typename Derived>
    MatX getDerivativeWpT(const Eigen::MatrixBase<Derived> &normal) const;

    template<typename Derived>
    void setWeight(const Eigen::MatrixBase<Derived> &Weight) {
        this->weight = Weight;
    }

 private:
    // updated by evaluation callback
    const Scalar *pt;
    const VecE<const MatX*> jacsw1, jacsw2;

    Mat3 weight = Mat3::Identity();

    const Scalar w1, w2;
};
}

#include "wave/odometry/geometry/impl/line_impl.hpp"

#endif  // WAVE_LINE_HPP
