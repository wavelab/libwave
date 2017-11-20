#ifndef WAVE_SE3_RESIDUALS_HPP
#define WAVE_SE3_RESIDUALS_HPP

// Contains point to line and point to plane residuals, but reformulated with interpolated SE3 objects
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

#endif  // WAVE_SE3_RESIDUALS_HPP
