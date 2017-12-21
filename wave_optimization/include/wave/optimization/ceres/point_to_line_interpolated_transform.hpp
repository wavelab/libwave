#ifndef WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP
#define WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "wave/geometry/transformation.hpp"

namespace wave {

class SE3PointToLine : public ceres::SizedCostFunction<3, 12> {
 private:
    const double *const pt;
    const double *const ptA;
    const double *const ptB;
    const double *const scale;
    Mat3 weight_matrix;

 public:
    virtual ~SE3PointToLine() {}
    SE3PointToLine(const double *const p, const double *const pA, const double *const pB, const double *const scal, Mat3 weight)
            : pt(p), ptA(pA), ptB(pB), scale(scal), weight_matrix(weight) {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};
}

#endif //WAVE_POINT_TO_LINE_INTERPOLATED_TRANSFORM_HPP
