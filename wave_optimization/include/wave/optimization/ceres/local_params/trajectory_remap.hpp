#ifndef WAVE_TRAJECTORY_REMAP_HPP
#define WAVE_TRAJECTORY_REMAP_HPP

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include "wave/geometry_og/transformation.hpp"

namespace wave {
/**
 * This parameterization is a bit of a hack because Ceres is dumb
 * and doesn't have an option to specify Jacobians directly wrt local parameterizations.
 *
 * The "lift jacobian" is just a matrix to eliminate extra rows in the "full jacobian"
 *
 * The trajectory state consists of one velocity state and N SE3 poses.
 */

template<int DIM>
class TrajectoryParamRemap : public ceres::LocalParameterization {
    using M_Type = Eigen::Matrix<double, DIM/2 + 3, DIM/2 + 3>;
    const M_Type p_mat;
 public:
    explicit TrajectoryParamRemap(M_Type p_mat) : p_mat(p_mat) {}
    virtual ~TrajectoryParamRemap() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return DIM; }
    virtual int LocalSize() const { return DIM/2 + 3; }
};
}

#include "wave/optimization/ceres/local_params/impl/trajectory_remap_impl.hpp"

#endif //WAVE_TRAJECTORY_REMAP_HPP
