#ifndef WAVE_FIXED_PLANE_HPP
#define WAVE_FIXED_PLANE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/odometry/geometry/assign_jacobians.hpp"

namespace wave {

template <typename Scalar, int... states>
class FixedPlaneResidual : public ceres::SizedCostFunction<1, 12, states...> {
 public:
    virtual ~FixedPlaneResidual() {}

    FixedPlaneResidual(const Scalar *pt,
                  VecE<const MatX*> &jacsw1,
                  VecE<const MatX*> &jacsw2,
                  const Scalar &w1,
                  const Scalar &w2,
                  const Vec6 &plane)
            : pt(pt),
              jacsw1(std::move(jacsw1)),
              jacsw2(std::move(jacsw2)),
              w1(w1),
              w2(w2),
              plane(plane) { }

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
    const Vec6 &plane;

    Eigen::Matrix<double, 1, 1> weight = Eigen::Matrix<double, 1, 1>::Identity();
};
}

#include "wave/odometry/geometry/impl/fixed_plane_impl.hpp"

#endif //WAVE_FIXED_PLANE_HPP
