#ifndef WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
#define WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class SE3PointToPlane : public ceres::SizedCostFunction<1, 12> {
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const ptC;
    const double *const scale;
    double weight;

 public:
    virtual ~SE3PointToPlane() {}
    SE3PointToPlane(const double *const p,
                    const double *const pA,
                    const double *const pB,
                    const double *const pC,
                    const double *const scal,
                    double weighting)
            : pt(p), ptA(pA), ptB(pB), ptC(pC), scale(scal), weight(weighting) {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif //WAVE_POINT_TO_PLANE_INTERPOLATED_TRANSFORM_HPP
