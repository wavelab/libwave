/**
 * The purpose of this parameterization is to only update solution in well constrained directions
 */
#ifndef WAVE_SOLUTION_REMAPPING_PARAMETERIZATION_HPP
#define WAVE_SOLUTION_REMAPPING_PARAMETERIZATION_HPP

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

namespace wave {

template<int Dimension>
class RemapParameterization : public ceres::LocalParameterization {
 private:
    using M_Type = Eigen::Matrix<double, Dimension, Dimension>;
    using V_Type = Eigen::Matrix<double, Dimension, 1>;
    M_Type projection_matrix;
 public:
    RemapParameterization(const M_Type projection_matrix) : projection_matrix(projection_matrix) {}
    virtual ~RemapParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        Eigen::Map<const V_Type> x_map(x);
        Eigen::Map<const V_Type> delta_map(delta);
        Eigen::Map<V_Type> x_plus_delta_map(x_plus_delta);

        x_plus_delta_map = x_map + this->projection_matrix * delta_map;
        return true;
    }
    virtual bool ComputeJacobian(const double* , double* jacobian) const {
        Eigen::Map<Eigen::Matrix<double, Dimension, Dimension, Eigen::RowMajor>> jac_map(jacobian);
        jac_map = this->projection_matrix;
        return true;
    }
    virtual int GlobalSize() const { return Dimension; }
    virtual int LocalSize() const { return Dimension; }
};

}

#endif //WAVE_SOLUTION_REMAPPING_PARAMETERIZATION_HPP
