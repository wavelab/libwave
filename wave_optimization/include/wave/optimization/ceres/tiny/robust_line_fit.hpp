#ifndef WAVE_ROBUST_LINE_FIT_HPP
#define WAVE_ROBUST_LINE_FIT_HPP

#include <ceres/tiny_solver.h>

#include "wave/utils/math.hpp"

namespace wave {

class RobustLineFit {
 private:
    mutable double *line_param;
    mutable double *opt_param;
    mutable Vec6 current_line_param;
    mutable Eigen::Matrix<double, 6, 4> cur_lift_jacobian;
    const std::vector<const float *> pts;
    const double k;

 public:
    RobustLineFit(double *line_param, double *opt_param, const std::vector<const float *> pts, const double &k)
        : line_param(line_param), opt_param(opt_param), pts(pts), k(k) {
        if (pts.size() < 3) {
            throw std::runtime_error("Trying to robustly fit a line to two or fewer points!");
        }
    }

    typedef double Scalar;
    enum {
        NUM_RESIDUALS = Eigen::Dynamic,
        NUM_PARAMETERS = 4
    };

    int NumResiduals() const {
        return static_cast<int>(this->pts.size());
    }

    bool operator()(const double *parameters, double *residuals, double *jacobian) const;
};
}

#endif  // WAVE_ROBUST_LINE_FIT_HPP
