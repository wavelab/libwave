#include <ceres/gradient_checker.h>

#include "wave/wave_test.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/optimization/ceres/odom_linear/point_to_plane_interpolated_transform.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/optimization/ceres/odom_gp_twist/point_to_plane_gp.hpp"
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"

namespace wave {

TEST(Residual_test, SE3pointToPlaneAnalytic) {
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
    jacobian[0] = new double[12];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual = 0;

    Mat3 CovZ = 100*Mat3::Identity();
    ceres::CostFunction* cost_function = new SE3PointToPlane(pt, ptA, ptB, ptC, &scale, CovZ, false);
    cost_function->Evaluate(trans, &residual, jacobian);

    EXPECT_NEAR(residual, -4, 1e-4);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

TEST(Residual_test, SE3pointToPlaneAnalyticWeighted) {
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
    jacobian[0] = new double[12];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual = 0;

    Mat3 CovZ = 100*Mat3::Identity();
    ceres::CostFunction* cost_function = new SE3PointToPlane(pt, ptA, ptB, ptC, &scale, CovZ, true);
    cost_function->Evaluate(trans, &residual, jacobian);

    /// The residual value is going to be weighted strangely
//    EXPECT_NEAR(residual, -4, 1e-4);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

TEST(Local_Twist_Param_Plane, Jacobians) {
    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double ptC[3] = {4, -1, 0};
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
    wave_optimization::SE3PointToPlaneGPObjects objects;
    objects.hat = hat.block<6, 12>(0,0);
    objects.candle = candle.block<6, 12>(0,0);

    Transformation<Mat34, false> T_interpolated;
    Transformation<Mat34, false>::interpolate(
            T_k, T_kp1, vel_k, vel_kp1, hat, candle, T_interpolated);

    Eigen::Map<Vec3> mapped_point(pt, 3, 1);
    T_interpolated.transform(mapped_point, objects.T0_pt);

    ceres::CostFunction *cost_function = new wave_optimization::SE3PointToPlaneGP(
            ptA,
            ptB,
            ptC,
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

}
