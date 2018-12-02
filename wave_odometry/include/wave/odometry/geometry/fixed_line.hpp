//
// Created by ben on 10/4/18.
//

#ifndef WAVE_FIXED_LINE_HPP
#define WAVE_FIXED_LINE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/geometry_og/transformation.hpp"
#include "wave/odometry/geometry/assign_jacobians.hpp"
#include "wave/odometry/geometry/rotate_error.hpp"

namespace wave {

/// error is 3 dimensional, followed by states: 12 dimensional vector for map transform, and then all the trajectory states
template <typename Scalar, int... states>
class FixedLineResidual : public ceres::SizedCostFunction<3, 12, states...> {
 public:
    virtual ~FixedLineResidual() {}

    FixedLineResidual(const Scalar *pt,
                 VecE<const MatX*> &jacsw1,
                 VecE<const MatX*> &jacsw2,
                 const Scalar &w1,
                 const Scalar &w2,
                 const Vec6 &line)
            : pt(pt),
              jacsw1(std::move(jacsw1)),
              jacsw2(std::move(jacsw2)),
              w1(w1),
              w2(w2),
              line(line) { }

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
    const Scalar w1, w2;
    const Vec6 &line;

    Mat3 weight = Mat3::Identity();
};
}

#include "wave/odometry/geometry/impl/fixed_line_impl.hpp"

#endif //WAVE_FIXED_LINE_HPP
