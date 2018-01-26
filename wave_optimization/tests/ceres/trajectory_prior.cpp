#include <ceres/ceres.h>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/optimization/ceres/trajectory_prior.hpp"
#include "wave/optimization/ceres/constant_velocity.hpp"
#include "wave/optimization/ceres/null_SE3_parameterization.hpp"

namespace wave {

/**
 * Test that a trajectory prior works correctly during optimization
 * Due to local parameterization hack, gradient checker does not work
 */
TEST(trajectory_prior, unary_factor) {
    Vec6 twist, prior_twist;
    twist << 0.1, -0.4, 0.3, 1, 10, -5;
    prior_twist << -0.3, 0.2, 0.5, -3, 2, -8;
    Transformation prior, initial;
    prior.setFromExpMap(twist);

    ceres::CostFunction *cost_function =
      new TrajectoryPrior(Eigen::Matrix<double, 12, 12>::Identity(), prior.inverse(), prior_twist);

    ceres::LocalParameterization *se3_param = new NullSE3Parameterization;

    ceres::Problem problem;
    problem.AddParameterBlock(initial.getInternalMatrix().data(), 12, se3_param);
    problem.AddParameterBlock(twist.data(), 6);

    problem.AddResidualBlock(cost_function, nullptr, initial.getInternalMatrix().data(), twist.data());

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport();

    auto diff = initial.manifoldMinus(prior);
    EXPECT_NEAR(diff.norm(), 0, 1e-8);
    EXPECT_NEAR((twist - prior_twist).norm(), 0, 1e-8);
}

/**
 * Test that a trajectory prior works well with multiple states
 * optimization problem will be initialized a little ways from the solution,
 * and it will converge back
 */
TEST(trajectory_prior, binary_factor) {
    const double delta_T = 0.1;
    Vec6 init_twist_prior, next_twist_prior, init_twist, next_twist;
    init_twist_prior << 0.1, -0.4, 0.3, 1, 10, -5;

    /// Need to constrain the initial state with a prior.
    /// In this case Identity to treat it as the origin.
    Vec6 prior_vel;
    prior_vel = init_twist_prior / delta_T;
    Transformation prior_initial, prior_next, initial, next;
    prior_initial.setIdentity();

    // Need to provide prior for binary factor
    prior_next.setFromExpMap(delta_T * prior_vel);

    // now invert the pose priors ahead of time to save a bit of computation during residual evaluation
    prior_initial.invert();
    prior_next.invert();

    ceres::CostFunction *cost_function =
      new TrajectoryPrior(10000*Eigen::Matrix<double, 12, 12>::Identity(), prior_initial, prior_vel);

    ceres::LocalParameterization *se3_param = new NullSE3Parameterization;

    ceres::Problem problem;
    problem.AddParameterBlock(initial.getInternalMatrix().data(), 12, se3_param);
    problem.AddParameterBlock(init_twist.data(), 6);
    problem.AddParameterBlock(next.getInternalMatrix().data(), 12, se3_param);
    problem.AddParameterBlock(next_twist.data(), 6);

    problem.AddResidualBlock(cost_function, nullptr, initial.getInternalMatrix().data(), init_twist.data());

    double tk = 0;
    double tkp1 = 0.1;
    double tau = 0;

    Mat6 Qc = Mat6::Identity();
    wave_kinematics::ConstantVelocityPrior::Mat12 transition, Q, weight;

    wave_kinematics::ConstantVelocityPrior cv_model(tk, tkp1, &tau, Qc);
    cv_model.calculateTransitionMatrix(transition, tk, tkp1);
    cv_model.calculateLinCovariance(Q, tk, tkp1);

    weight = Q.inverse().sqrt();

    ceres::CostFunction *bin_cost_function = new ConstantVelocityPrior(weight, tkp1);

    init_twist.setZero();
    initial.setIdentity();
    next_twist.setZero();
    next.setIdentity();

    problem.AddResidualBlock(bin_cost_function,
                             nullptr,
                             initial.getInternalMatrix().data(),
                             next.getInternalMatrix().data(),
                             init_twist.data(),
                             next_twist.data());

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport();

    auto init_diff = (initial * prior_initial).logMap();
    auto next_diff = (next * prior_next).logMap();
    auto init_vel_diff = init_twist - prior_vel;
    auto next_vel_diff = next_twist - prior_vel;

    EXPECT_NEAR(init_diff.norm(), 0, 1e-6);
    EXPECT_NEAR(next_diff.norm(), 0, 1e-6);
    EXPECT_NEAR(init_vel_diff.norm(), 0, 1e-6);
    EXPECT_NEAR(next_vel_diff.norm(), 0, 1e-6);
}
}