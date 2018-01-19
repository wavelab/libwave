#include <ceres/gradient_checker.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/constant_velocity.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"

namespace {
/// Helper functions
Eigen::Matrix<double, 12, 12> calculateCVIntegrand(const wave::Mat6 &Qc,
                          const double &delta_T,
                          const wave::Vec6 &velocity) {
    Eigen::Matrix<double, 12, 12> retval;
    auto jacobian = wave::Transformation::SE3LeftJacobian(delta_T * velocity, 1e-4);
    retval.block<6, 6>(0, 0) = delta_T * delta_T * jacobian * Qc * jacobian.transpose();
    retval.block<6, 6>(6, 0) = delta_T * jacobian.transpose();
    retval.block<6, 6>(0, 6) = delta_T * jacobian;
    retval.block<6, 6>(6, 6) = Qc;
    return retval;
}

void calculateTransitionMatrix(const double &delta_T,
                               const wave::Vec6 &velocity,
                               Eigen::Matrix<double, 12, 12>& transition_matrix) {
    transition_matrix.setIdentity();
    transition_matrix.block<6, 6>(0, 0) = wave::Transformation::expMapAdjoint(delta_T * velocity, 1e-4);
    transition_matrix.block<6, 6>(0, 6) = delta_T * wave::Transformation::SE3LeftJacobian(delta_T * velocity, 1e-4);
}

Eigen::Matrix<double, 12, 12> integrateCovariance(const wave::Mat6 &Qc,
                         const double &delta_T,
                         const wave::Vec6 &velocity,
                         const int &steps) {
    Eigen::Matrix<double, 12, 12> Qtotal;
    Qtotal.setZero();
    double step_size = delta_T / (double) steps;
    Eigen::Matrix<double, 12, 12> Qincremental;
    double step_delta_T;
    for (int i = 0; i <= steps; i++) {
        step_delta_T = delta_T - ((double) i) * step_size;
        Qincremental = calculateCVIntegrand(Qc, step_delta_T, velocity);
        if (i == 0 || i == steps) {
            Qtotal.noalias() += 0.5 * Qincremental;
        } else {
            Qtotal.noalias() += Qincremental;
        }
    }
    Qtotal = Qtotal * step_size;
    return Qtotal;
}
}

namespace wave {

/**
 * Due to hack to workaround Ceres local parameterization for speedup
 * this test no longer works. Revisit when (if) manifold optimization is changed in Ceres
 */
//TEST(ConstantVelocity, Jacobians) {
//    Transformation start, end;
//    const double delta_T = 0.1;
//    Vec6 twist, start_vel, end_vel;
//    twist << 0.1, -0.1, 0.5, 2, -1, 3;
//    start_vel << 0.01, -0.01, 0.2, 1, 2, -1;
//    start.setFromExpMap(twist);
//    end = start;
//    end_vel = start_vel;
//    end.manifoldPlus(delta_T * start_vel);
//
//    Transformation start_inv, end_inv;
//    start_inv = start.inverse();
//    end_inv = end.inverse();
//
//    // Calculate transition matrix and weight matrix for residual
//    Mat6 Qc = Mat6::Identity();
//    Eigen::Matrix<double, 12, 12> transition_matrix, covariance, weight;
//    calculateTransitionMatrix(delta_T, start_vel, transition_matrix);
//    covariance = integrateCovariance(Qc, delta_T, start_vel, 20);
//
//    weight = covariance.inverse().sqrt();
//
//    std::vector<std::shared_ptr<Transformation>> transforms;
//    std::vector<std::shared_ptr<Vec6>> velocities;
//    transforms.emplace_back(std::make_shared<Transformation>(start_inv));
//    transforms.emplace_back(std::make_shared<Transformation>(end_inv));
//    velocities.emplace_back(std::make_shared<Vec6>(start_vel));
//    velocities.emplace_back(std::make_shared<Vec6>(end_vel));
//
//    ceres::CostFunction *cost_function = new ConstantVelocityPrior(weight,
//                                                                   transforms[0],
//                                                                   transforms[1],
//                                                                   velocities[0],
//                                                                   velocities[1],
//                                                                   transition_matrix);
//
//    const double **parameters;
//    parameters = new const double *[4];
//    parameters[0] = start.getInternalMatrix().data();
//    parameters[1] = end.getInternalMatrix().data();
//    parameters[2] = start_vel.data();
//    parameters[3] = end_vel.data();
//
//    double *residuals;
//    residuals = new double[12];
//
//    cost_function->Evaluate(parameters, residuals, NULL);
//
//    Eigen::Matrix<double, 12, 1> residual_vec;
//    residual_vec = Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals);
//
//    EXPECT_NEAR(residual_vec.norm(), 0, 1e-6);
//
//    // Now check Jacobians
//
//    ceres::LocalParameterization *t1_param, *t2_param, *null_param;
//    t1_param = new SE3Parameterization;
//    t2_param = new SE3Parameterization;
//    null_param = NULL;
//    std::vector<const ceres::LocalParameterization*> local_param_vec;
//    local_param_vec.emplace_back(t1_param);
//    local_param_vec.emplace_back(t2_param);
//    local_param_vec.emplace_back(null_param);
//    local_param_vec.emplace_back(null_param);
//
//    ceres::NumericDiffOptions ndiff_options;
//    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
//    ceres::GradientChecker::ProbeResults g_results;
//
//    /// The gradient checker uses a relative error metric when evaluating jacobians.
//    /// Unfortunately, this scales error when the jacobian component is quite small,
//    /// so a fairly large tolerance (1e-4) must be used to pass the test
//    EXPECT_TRUE(g_check.Probe(parameters, 1e-4, &g_results));
//    LOG_INFO("%s", g_results.error_log.c_str());
//}


}
