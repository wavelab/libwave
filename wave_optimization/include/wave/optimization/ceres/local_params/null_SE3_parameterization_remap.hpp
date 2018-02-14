#ifndef WAVE_NULL_SE3_PARAMETERIZATION_REMAP_HPP
#define WAVE_NULL_SE3_PARAMETERIZATION_REMAP_HPP

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

namespace wave {

/**
 * This parameterization is a bit of a hack because Ceres is dumb
 * and doesn't have an option to specify Jacobians directly wrt local parameterizations.
 *
 * The "lift jacobian" is just a matrix to eliminate extra rows in the "full jacobian"
 */

class NullSE3ParameterizationRemap : public ceres::LocalParameterization {
    using M_Type = Eigen::Matrix<double, 6, 6>;
    const M_Type p_mat;
 public:
    explicit NullSE3ParameterizationRemap(M_Type p_mat) : p_mat(p_mat) {}
    virtual ~NullSE3ParameterizationRemap() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 12; }
    virtual int LocalSize() const { return 6; }
};

}

#endif //WAVE_NULL_SE3_PARAMETERIZATION_REMAP_HPP
