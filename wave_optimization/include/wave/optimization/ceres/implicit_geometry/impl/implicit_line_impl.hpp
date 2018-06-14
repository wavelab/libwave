#ifndef WAVE_IMPLICIT_LINE_IMPL_HPP
#define WAVE_IMPLICIT_LINE_IMPL_HPP

#include "wave/optimization/ceres/implicit_geometry/implicit_line.hpp"

namespace wave {

template<int cnt, int... idx>
bool ImplicitLineResidual::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Vec3> normal(&parameters[0][0]);
    std::vector<Eigen::Map<const Vec12>> states;
    for (int i = 0; i < cnt; i++) {
        states.emplace_back(Eigen::Map<const Vec12>(&parameters[i][0]));
    }

}

}

#endif //WAVE_IMPLICIT_LINE_IMPL_HPP
