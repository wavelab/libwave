#ifndef WAVE_POINT_TO_POINT_RESIDUAL_HPP
#define WAVE_POINT_TO_POINT_RESIDUAL_HPP

#include <ceres/ceres.h>
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"

namespace wave {

class AnalyticalPointToPoint : public ceres::SizedCostFunction<3, 12> {
 private:
    const double *const P1;
    const double *const P2;

 public:
    virtual ~AnalyticalPointToPoint(){};
    AnalyticalPointToPoint(const double *const p1, const double *const p2) : P1(p1), P2(p2) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};

class AnalyticalPointToPointInterpolated : public ceres::SizedCostFunction<3, 12> {
 private:
    const double *const P1;
    const double *const P2;
    const double *const alpha;

 public:
    virtual ~AnalyticalPointToPointInterpolated(){};
    AnalyticalPointToPointInterpolated(const double *const p1, const double *const p2, const double *const al)
        : P1(p1), P2(p2), alpha(al) {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};

}

#endif  // WAVE_POINT_TO_POINT_RESIDUAL_HPP
