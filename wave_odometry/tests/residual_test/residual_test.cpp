#include <ceres/solver.h>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/geometry/fixed_line.hpp"
#include "wave/odometry/geometry/fixed_plane.hpp"
#include "wave/odometry/geometry/line.hpp"
#include "wave/odometry/geometry/plane.hpp"
#include "wave/optimization/ceres/local_params/line_parameterization.hpp"
#include "wave/optimization/ceres/local_params/plane_parameterization.hpp"
#include "wave/utils/math.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(line_residual, jacobian) {
    Vec3 state_point;
    state_point.setRandom();
    VecE<MatX> jac1, jac2;
    double w1 = 1.0;
    double w2 = 0.0;
    jac1.emplace_back(MatX(6,3));
    jac2.emplace_back(MatX(6,3));
    jac1.front().setZero();
    jac2.front().setZero();
    jac1.front().block<3,3>(3,0).setIdentity();

    Vec6 line_param;
    line_param.setRandom();
    line_param.block<3,1>(0,0).normalize();

    VecE<const MatX*> jacs1, jacs2;
    jacs1.emplace_back(&(jac1.front()));
    jacs2.emplace_back(&(jac2.front()));

    LineResidual<double, 3> line_residual(state_point.data(), jacs1, jacs2, w1, w2);

    double *params[2];
    params[0] = line_param.data();
    params[1] = state_point.data();

    Vec3 error;

    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> line_jacobian_analytic, line_jacobian_numeric;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> pt_jacobian_analytic, pt_jacobian_numeric;

    double *jacobians[2];
    jacobians[0] = line_jacobian_analytic.data();
    jacobians[1] = pt_jacobian_analytic.data();

    line_residual.Evaluate(params, error.data(), jacobians);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec3 saved_point = state_point;
    Vec3 perturbed_error;
    for (uint32_t i = 0; i < 3; ++i) {
        state_point = saved_point;
        state_point(i) += step;

        line_residual.Evaluate(params, perturbed_error.data(), nullptr);

        pt_jacobian_numeric.col(i) = (perturbed_error - error) / step;
    }
    state_point = saved_point;

    Vec6 saved_line = line_param;
    for (uint32_t i = 0; i < 6; ++i) {
        line_param = saved_line;
        line_param(i) += step;

        line_residual.Evaluate(params, perturbed_error.data(), nullptr);

        line_jacobian_numeric.col(i) = (perturbed_error - error) / step;
    }

    MatX pt_error = pt_jacobian_numeric - pt_jacobian_analytic;
    MatX line_error = line_jacobian_numeric - line_jacobian_analytic;

    EXPECT_NEAR(pt_error.norm(), 0, 1e-7);
    EXPECT_NEAR(line_error.norm(), 0, 1e-7);
}

TEST(fixed_line_residual, jacobian) {
    Vec3 state_point;
    state_point.setRandom();
    VecE<MatX> jac1, jac2;
    double w1 = 1.0;
    double w2 = 0.0;
    jac1.emplace_back(MatX(6,3));
    jac2.emplace_back(MatX(6,3));
    jac1.front().setZero();
    jac2.front().setZero();
    jac1.front().block<3,3>(3,0).setIdentity();

    Vec6 line_param;
    line_param.setRandom();
    line_param.block<3,1>(0,0).normalize();

    Transformation<> T_OL;

    VecE<const MatX*> jacs1, jacs2;
    jacs1.emplace_back(&(jac1.front()));
    jacs2.emplace_back(&(jac2.front()));

    FixedLineResidual<double, 3> residual(state_point.data(), jacs1, jacs2, w1, w2, line_param);

    double *params[2];
    params[0] = T_OL.storage.data();
    params[1] = state_point.data();

    Vec3 error;
    double *residuals = error.data();

    Eigen::Matrix<double, 3, 12, Eigen::RowMajor> T_OL_jacobian_analytic, T_OL_jacobian_numeric;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> pt_jacobian_analytic, pt_jacobian_numeric;
    T_OL_jacobian_numeric.setZero();

    double *jacobians[2];
    jacobians[0] = T_OL_jacobian_analytic.data();
    jacobians[1] = pt_jacobian_analytic.data();

    residual.Evaluate(params, residuals, jacobians);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec3 saved_point = state_point;
    Vec3 perturbed_error;
    for (uint32_t i = 0; i < 3; ++i) {
        state_point = saved_point;
        state_point(i) += step;

        residual.Evaluate(params, perturbed_error.data(), nullptr);

        pt_jacobian_numeric.col(i) = (perturbed_error - error) / step;
    }
    state_point = saved_point;

    Transformation<> saved_T_OL = T_OL;
    Vec6 perturb;
    for (uint32_t i = 0; i < 6; ++i) {
        T_OL = saved_T_OL;
        perturb.setZero();
        perturb(i) = step;
        T_OL.manifoldPlus(perturb);

        residual.Evaluate(params, perturbed_error.data(), nullptr);

        T_OL_jacobian_numeric.col(i) = (perturbed_error - error) / step;
    }

    MatX pt_error = pt_jacobian_numeric - pt_jacobian_analytic;
    MatX line_error = T_OL_jacobian_numeric - T_OL_jacobian_analytic;

    EXPECT_NEAR(pt_error.norm(), 0, 1e-7);
    EXPECT_NEAR(line_error.norm(), 0, 1e-7);
}

TEST(plane_residual, jacobian) {
    Vec3 state_point;
    state_point.setRandom();
    VecE<MatX> jac1, jac2;
    double w1 = 1.0;
    double w2 = 0.0;
    jac1.emplace_back(MatX(6,3));
    jac2.emplace_back(MatX(6,3));
    jac1.front().setZero();
    jac2.front().setZero();
    jac1.front().block<3,3>(3,0).setIdentity();

    Vec6 line_param;
    line_param.setRandom();
    line_param.block<3,1>(0,0).normalize();

    VecE<const MatX*> jacs1, jacs2;
    jacs1.emplace_back(&(jac1.front()));
    jacs2.emplace_back(&(jac2.front()));

    PlaneResidual<double, 3> plane_residual(state_point.data(), jacs1, jacs2, w1, w2);

    double *params[2];
    params[0] = line_param.data();
    params[1] = state_point.data();

    double residuals[1];

    Eigen::Matrix<double, 1, 6> line_jacobian_analytic, line_jacobian_numeric;
    Eigen::Matrix<double, 1, 3> pt_jacobian_analytic, pt_jacobian_numeric;

    double *jacobians[2];
    jacobians[0] = line_jacobian_analytic.data();
    jacobians[1] = pt_jacobian_analytic.data();

    plane_residual.Evaluate(params, residuals, jacobians);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec3 saved_point = state_point;
    double perturbed_error;
    for (uint32_t i = 0; i < 3; ++i) {
        state_point = saved_point;
        state_point(i) += step;

        plane_residual.Evaluate(params, &perturbed_error, nullptr);

        pt_jacobian_numeric(i) = (perturbed_error - residuals[0]) / step;
    }
    state_point = saved_point;

    Vec6 saved_line = line_param;
    for (uint32_t i = 0; i < 6; ++i) {
        line_param = saved_line;
        line_param(i) += step;

        plane_residual.Evaluate(params, &perturbed_error, nullptr);

        line_jacobian_numeric(i) = (perturbed_error - residuals[0]) / step;
    }

    MatX pt_error = pt_jacobian_numeric - pt_jacobian_analytic;
    MatX line_error = line_jacobian_numeric - line_jacobian_analytic;

    EXPECT_NEAR(pt_error.norm(), 0, 1e-6);
    EXPECT_NEAR(line_error.norm(), 0, 1e-6);
}

TEST(fixed_plane_residual, jacobian) {
    Vec3 state_point;
    state_point.setRandom();
    VecE<MatX> jac1, jac2;
    double w1 = 1.0;
    double w2 = 0.0;
    jac1.emplace_back(MatX(6,3));
    jac2.emplace_back(MatX(6,3));
    jac1.front().setZero();
    jac2.front().setZero();
    jac1.front().block<3,3>(3,0).setIdentity();

    Vec6 line_param;
    line_param.setRandom();
    line_param.block<3,1>(0,0).normalize();

    Transformation<> T_OL;

    VecE<const MatX*> jacs1, jacs2;
    jacs1.emplace_back(&(jac1.front()));
    jacs2.emplace_back(&(jac2.front()));

    FixedPlaneResidual<double, 3> plane_residual(state_point.data(), jacs1, jacs2, w1, w2, line_param);

    double *params[2];
    params[0] = T_OL.storage.data();
    params[1] = state_point.data();

    double residuals[1];

    Eigen::Matrix<double, 1, 12> T_OL_jacobian_analytic, T_OL_jacobian_numeric;
    Eigen::Matrix<double, 1, 3> pt_jacobian_analytic, pt_jacobian_numeric;
    T_OL_jacobian_numeric.setZero();

    double *jacobians[2];
    jacobians[0] = T_OL_jacobian_analytic.data();
    jacobians[1] = pt_jacobian_analytic.data();

    plane_residual.Evaluate(params, residuals, jacobians);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec3 saved_point = state_point;
    double perturbed_error;
    for (uint32_t i = 0; i < 3; ++i) {
        state_point = saved_point;
        state_point(i) += step;

        plane_residual.Evaluate(params, &perturbed_error, nullptr);

        pt_jacobian_numeric(i) = (perturbed_error - residuals[0]) / step;
    }
    state_point = saved_point;

    Transformation<> saved_T_OL = T_OL;
    Vec6 perturb;
    for (uint32_t i = 0; i < 6; ++i) {
        perturb.setZero();
        perturb(i) = step;
        T_OL = saved_T_OL;
        T_OL.manifoldPlus(perturb);

        plane_residual.Evaluate(params, &perturbed_error, nullptr);

        T_OL_jacobian_numeric(i) = (perturbed_error - residuals[0]) / step;
    }

    MatX pt_error = pt_jacobian_numeric - pt_jacobian_analytic;
    MatX line_error = T_OL_jacobian_numeric - T_OL_jacobian_analytic;

    EXPECT_NEAR(pt_error.norm(), 0, 1e-6);
    EXPECT_NEAR(line_error.norm(), 0, 1e-6);
}

}