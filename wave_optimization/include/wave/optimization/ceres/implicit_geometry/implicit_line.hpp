/**
 * This residual penalizes the distance of a number of points to a line fit to those points
 */

#ifndef WAVE_IMPLICIT_LINE_HPP
#define WAVE_IMPLICIT_LINE_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "wave/optimization/ceres/implicit_geometry/point_meta_data.hpp"
#include "wave/geometry/transformation.hpp"

namespace wave {

template <int cnt, int... num>
class ImplicitLineResidual : public ceres::SizedCostFunction<cnt, num...> {
 public:
    virtual ~ImplicitLineResidual() {}

    ImplicitLineResidual(const std::vector<Vec3f> &pts_data, const std::vector<PtMetaData> &pts_info)
        : pts_data(pts_data), pts_info(pts_info) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

 private:
    const std::vector<Vec3f> &pts_data;
    const std::vector<PtMetaData> &pts_info;
};
}

#include "wave/optimization/ceres/implicit_geometry/impl/implicit_line_impl.hpp"

#endif  // WAVE_IMPLICIT_LINE_HPP
