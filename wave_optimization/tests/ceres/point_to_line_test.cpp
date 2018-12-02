#include <ceres/gradient_checker.h>

#include "wave/wave_test.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/optimization/ceres/odom_linear/point_to_line_interpolated_transform.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp.hpp"
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"

namespace wave {

TEST(Residual_test, SE3pointToLineAnalytic) {
    const double **trans;
    trans = new const double *[1];
    trans[0] = new const double[12]{0.999613604886095,
                                    0.027796419313034,
                                    0,
                                    -0.027796419313034,
                                    0.999613604886095,
                                    0,
                                    0,
                                    0,
                                    1,
                                    3.599536313918120,
                                    0.050036777340220,
                                    0};

    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[24];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual[2] = {0};

    ceres::CostFunction* cost_function = new SE3PointToLine(pt, ptA, ptB, &scale, Mat3::Identity(), false);
    cost_function->Evaluate(trans, residual, jacobian);

    EXPECT_NEAR(std::sqrt(residual[0]*residual[0] + residual[1]*residual[1]), 2.295096987523390, 1e-4);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1.1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

TEST(Residual_test, SE3pointToLineAnalyticWeighted) {
    const double **trans;
    trans = new const double *[1];
    trans[0] = new const double[12]{0.999613604886095,
                                    0.027796419313034,
                                    0,
                                    -0.027796419313034,
                                    0.999613604886095,
                                    0,
                                    0,
                                    0,
                                    1,
                                    3.599536313918120,
                                    0.050036777340220,
                                    0};

    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[24];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual[2] = {0};

    ceres::CostFunction* cost_function = new SE3PointToLine(pt, ptA, ptB, &scale, 100*Mat3::Identity(), false);
    cost_function->Evaluate(trans, residual, jacobian);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1.1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

TEST(Local_Twist_Param_Line, Jacobians) {
    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};

    const double delta_T = 0.5;
    const double **params;
    params = new const double *[4];

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 vel_k, vel_kp1;
    
    Vec12 state_k_eps, state_kp1_eps;

    state_k_eps.setZero();
    state_kp1_eps.setZero();

    state_k_eps(3) = 0.2;

    params[0] = state_k_eps.data();
    params[1] = state_kp1_eps.data();

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.1, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1 = T_k;
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);
    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);
    wave_optimization::SE3PointToLineGPObjects objects;
    objects.hat = hat.block<6, 12>(0,0);
    objects.candle = candle.block<6, 12>(0,0);

    Transformation<Mat34, false> T_interpolated;
    Transformation<Mat34, false>::interpolate(
            T_k, T_kp1, vel_k, vel_kp1, hat, candle, T_interpolated);

    Eigen::Map<Vec3> mapped_point(pt, 3, 1);
    T_interpolated.transform(mapped_point, objects.T0_pt);

    ceres::CostFunction *cost_function = new wave_optimization::SE3PointToLineGP(
                                                              ptA,
                                                              ptB,
                                                              objects,
                                                              Mat3::Identity(),
                                                              true);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, nullptr, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(params, 1.1e-6, &g_results));
    LOG_INFO("Max relative error was %f", g_results.maximum_relative_error);

    for(unsigned int i = 0; i < g_results.jacobians.size(); i++) {
        std::cout << "Analytic Jacobian: \n" << g_results.jacobians.at(i) << "\n";
        std::cout << "Numeric Jacobian: \n" << g_results.numeric_jacobians.at(i) << "\n";
    }
}

// This test is to figure out how many iterations are required for the approximate interpolated transform
// to be close to the analytical version
TEST(Local_Twist_Param, Interp_Approx) {
    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 epsT_k, epsT_kp1, vel_k, vel_kp1, eps_vel_k, eps_vel_kp1;

    eps_vel_k << 0, 0, 0, 0.1, 0, 0;
    eps_vel_kp1 << 0, 0, 0, 0, 0, 0;
    epsT_k << 0, 0.1, 0, 0.4, 0, 0;
    epsT_kp1 << 0, 0, 0, 0, 0, 0.4;

    double delta_T = 0.5;

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.1, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1 = T_k;
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;

    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);
    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    Transformation<Mat34, false> T_interpolated_op, T_interpolated_analytical, T_interpolated_approx;
    Transformation<Mat34, false>::interpolate(
            T_k, T_kp1, vel_k, vel_kp1, hat, candle, T_interpolated_op);

    T_interpolated_approx = T_interpolated_op;

    Vec6 update_vector = hat.block<6, 6>(0, 0) * epsT_k +
                         hat.block<6, 6>(0, 6) * eps_vel_k +
                         candle.block<6, 6>(0, 0) * epsT_kp1 +
                         candle.block<6, 6>(0, 6) * eps_vel_kp1;
    T_interpolated_approx.manifoldPlus(update_vector);

    T_k.manifoldPlus(epsT_k);
    T_kp1.manifoldPlus(epsT_kp1);
    vel_k = vel_k + eps_vel_k;
    vel_kp1 = vel_kp1 + eps_vel_kp1;

    Transformation<Mat34, false>::interpolate(
            T_k, T_kp1, vel_k, vel_kp1, hat, candle, T_interpolated_analytical);

    Vec6 manifold_difference = T_interpolated_analytical.manifoldMinus(T_interpolated_approx);

    // This is kind of a weak test, but given that error when the update vector is 0, if the error
    // is less than the update vector otherwise, enough outer iterations will drive this down.
    EXPECT_TRUE(manifold_difference.norm() < update_vector.norm());
}

}
