#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/gtsam/motion_factor.hpp"
#include "wave/gtsam/pose_vel_bias.hpp"
#include "wave/gtsam/pose_vel_imubias.hpp"
#include "wave/gtsam/pose_vel.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(pose_vel_state, example) {
    uint64_t from, to;
    double delta_t = 0.1;
    from = 1;
    to = 2;

    Eigen::Matrix<double, 12, 12> info;
    info.setIdentity();

    auto noise = gtsam::noiseModel::Gaussian::Information(info);
    MotionFactor<wave::PoseVel, wave::PoseVel> factor(from, to, delta_t, noise);

    PoseVel Start, End;
    Start.vel << 0, 0, 0.1, 5, 0, 0;
    End.vel << 0, 0, 0.1, 5, 0, 0;

    End.pose = Start.pose.Retract(delta_t * Start.vel);

    gtsam::Matrix H1, H2;
    Eigen::Matrix<double, 12, 1> err = factor.evaluateError(Start, End, H1, H2);

    EXPECT_TRUE(err.isZero());
}

TEST(pose_vel_state, single_prior) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    PoseVel state;
    initial.insert(1, state);

    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;

    Eigen::Matrix<double, 12, 12> info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    auto factor = gtsam::PriorFactor<PoseVel>(1, state, model);
    graph.add(factor);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    EXPECT_TRUE(gtsam::traits<PoseVel>::Equals(state, result.at<PoseVel>(1)));
}

TEST(pose_vel_state, logmap) {
    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    PoseVel state;
    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;

    gtsam::Matrix H_combined, H_pose, H_vel;

    // Expected values from separate objects
    auto tangent_pose = gtsam::Pose3::Logmap(state.pose, H_pose);
    auto tangent_vel = gtsam::traits<gtsam::Vector6>::Logmap(state.vel, H_vel);

    // Actual value
    auto tangent_combined = gtsam::traits<PoseVel>::Logmap(state, H_combined);

    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(0), tangent_pose);
    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(6), tangent_vel);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(0, 0)), H_pose);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(6, 6)), H_vel);
    EXPECT_TRUE((H_combined.block<6, 6>(0, 6)).isZero());
}

TEST(pose_vel_state, trivial_problem) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    const double delta_t = 0.1;

    std::vector<PoseVel> states(10);
    initial.insert(0, states.at(0));
    states.at(0).vel << 0, 0, 0.1, 5, 0, 0;
    // Add prior factor on first state
    auto info = Eigen::Matrix<double, 12, 12>::Identity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    graph.add(gtsam::PriorFactor<wave::PoseVel>(0, states.at(0), model));

    for (uint16_t i = 1; i < states.size(); i++) {
        initial.insert(i, states.at(i));
        states.at(i).pose =
          states.at(i - 1).pose.retract(delta_t * states.at(i - 1).vel);
        states.at(i).vel = states.at(i - 1).vel;
        graph.add(
          MotionFactor<wave::PoseVel, wave::PoseVel>(i - 1, i, delta_t, model));
    }

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    for (uint64_t i = 0; i < result.size(); i++) {
        PoseVel res = result.at<PoseVel>(i);
        EXPECT_TRUE(gtsam::traits<PoseVel>::Equals(states.at(i), res, 1e-3));
    }
}

TEST(pose_vel_bias_state, example) {
    uint64_t from, to;
    double delta_t = 0.1;
    from = 1;
    to = 2;

    Eigen::Matrix<double, 15, 15> info;
    info.setIdentity();
    auto noise = gtsam::noiseModel::Gaussian::Information(info);
    MotionFactor<wave::PoseVelBias, wave::PoseVelBias> factor(
      from, to, delta_t, noise);

    PoseVelBias Start, End;
    Start.vel << 0, 0, 0.1, 5, 0, 0;
    End.vel << 0, 0, 0.1, 5, 0, 0;

    End.pose = Start.pose.Retract(delta_t * Start.vel);

    gtsam::Matrix H1, H2;
    Eigen::Matrix<double, 15, 1> err = factor.evaluateError(Start, End, H1, H2);

    EXPECT_TRUE(err.isZero());
}

TEST(pose_vel_bias_state, single_prior) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    PoseVelBias state;
    initial.insert(1, state);

    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;
    state.bias << 0.2, -0.1, 0.4;

    Eigen::Matrix<double, 15, 15> info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    auto factor = gtsam::PriorFactor<PoseVelBias>(1, state, model);
    graph.add(factor);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    EXPECT_TRUE(
      gtsam::traits<PoseVelBias>::Equals(state, result.at<PoseVelBias>(1)));
}

TEST(pose_vel_bias_state, logmap) {
    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    PoseVelBias state;
    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;
    state.bias << 0.2, -0.1, 0.4;

    gtsam::Matrix H_combined, H_pose, H_vel, H_bias;

    // Expected values from separate objects
    auto tangent_pose = gtsam::Pose3::Logmap(state.pose, H_pose);
    auto tangent_vel = gtsam::traits<gtsam::Vector6>::Logmap(state.vel, H_vel);
    auto tangent_bias =
      gtsam::traits<gtsam::Vector3>::Logmap(state.bias, H_bias);

    // Actual value
    auto tangent_combined =
      gtsam::traits<PoseVelBias>::Logmap(state, H_combined);

    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(0), tangent_pose);
    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(6), tangent_vel);
    EXPECT_PRED2(VectorsNear, tangent_combined.segment<3>(12), tangent_bias);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(0, 0)), H_pose);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(6, 6)), H_vel);
    EXPECT_PRED2(MatricesNear, (H_combined.block<3, 3>(12, 12)), H_bias);
    EXPECT_TRUE((H_combined.block<6, 9>(0, 6)).isZero());
}

TEST(pose_vel_bias_state, trivial_problem) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    const double delta_t = 0.1;

    std::vector<PoseVelBias> states(10);
    initial.insert(0, states.at(0));
    states.at(0).vel << 0, 0, 0.1, 5, 0, 0;
    // Add prior factor on first state
    auto info = Eigen::Matrix<double, 15, 15>::Identity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    graph.add(gtsam::PriorFactor<PoseVelBias>(0, states.at(0), model));

    for (uint16_t i = 1; i < states.size(); i++) {
        initial.insert(i, states.at(i));
        states.at(i).pose =
          states.at(i - 1).pose.retract(delta_t * states.at(i - 1).vel);
        states.at(i).vel = states.at(i - 1).vel;
        graph.add(MotionFactor<wave::PoseVelBias, wave::PoseVelBias>(
          i - 1, i, delta_t, model));
    }

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    for (uint64_t i = 0; i < result.size(); i++) {
        PoseVelBias res = result.at<PoseVelBias>(i);
        EXPECT_TRUE(
          gtsam::traits<PoseVelBias>::Equals(states.at(i), res, 1e-3));
    }
}

TEST(pose_vel_imubias_state, example) {
    uint64_t from, to;
    double delta_t = 0.1;
    from = 1;
    to = 2;

    Eigen::Matrix<double, 18, 18> info;
    info.setIdentity();

    auto noise = gtsam::noiseModel::Gaussian::Information(info);
    MotionFactor<wave::PoseVelImuBias, wave::PoseVelImuBias> factor(
                                                            from,
                                                            to,
                                                            delta_t,
                                                            noise);

    PoseVelImuBias Start, End;
    Start.vel << 0, 0, 0.1, 5, 0, 0;
    End.vel << 0, 0, 0.1, 5, 0, 0;

    End.pose = Start.pose.Retract(delta_t * Start.vel);

    gtsam::Matrix H1, H2;
    Eigen::Matrix<double, 15, 1> err = factor.evaluateError(Start, End, H1, H2);

    EXPECT_TRUE(err.isZero());
}

TEST(pose_vel_imubias_state, single_prior) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    PoseVelImuBias state;
    initial.insert(1, state);

    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;

    Eigen::Matrix<double, 18, 18> info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    auto factor = gtsam::PriorFactor<PoseVelImuBias>(1, state, model);
    graph.add(factor);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    EXPECT_TRUE(gtsam::traits<PoseVelImuBias>::Equals(
                                                state,
                                                result.at<PoseVelImuBias>(1)));
}

TEST(pose_vel_imubias_state, logmap) {
    gtsam::Vector biasAcc(gtsam::Vector3(0.2, -0.1, 0));
    gtsam::Vector biasGyro(gtsam::Vector3(0.1, -0.3, -0.2));
    gtsam::imuBias::ConstantBias bias(biasAcc, biasGyro);

    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    PoseVelImuBias state;
    gtsam::Pose3 prior_pose(T_local_s1.matrix());
    state.pose = prior_pose;
    state.vel << -0.4, 0.3, 0.1, 5, 2, 1;
    state.imu_bias = bias;
    gtsam::Matrix H_combined, H_pose, H_vel, H_imubias;

    // Expected values from separate objects
    auto tangent_pose = gtsam::Pose3::Logmap(state.pose, H_pose);
    auto tangent_vel = gtsam::traits<gtsam::Vector6>::Logmap(state.vel, H_vel);
    auto tangent_imubias = gtsam::traits<gtsam::imuBias::ConstantBias>::Logmap(
                                                            bias,
                                                            H_imubias);
    // Actual value
    auto tangent_combined = gtsam::traits<PoseVelImuBias>::Logmap(
                                                            state,
                                                            H_combined);

    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(0), tangent_pose);
    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(6), tangent_vel);
    EXPECT_PRED2(VectorsNear, tangent_combined.segment<6>(12), tangent_imubias);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(0, 0)), H_pose);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(6, 6)), H_vel);
    EXPECT_PRED2(MatricesNear, (H_combined.block<6, 6>(12, 12)), H_imubias);
    EXPECT_TRUE((H_combined.block<6, 6>(0, 6)).isZero());
}

TEST(pose_vel_imubias_state, trivial_problem) {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    const double delta_t = 0.1;

    std::vector<PoseVelImuBias> states(10);
    initial.insert(0, states.at(0));
    states.at(0).vel << 0, 0, 0.1, 5, 0, 0;
    // Add prior factor on first state
    auto info = Eigen::Matrix<double, 18, 18>::Identity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    graph.add(gtsam::PriorFactor<wave::PoseVelImuBias>(0, states.at(0), model));

    for (uint16_t i = 1; i < states.size(); i++) {
        initial.insert(i, states.at(i));
        states.at(i).pose =
          states.at(i - 1).pose.retract(delta_t * states.at(i - 1).vel);
        states.at(i).vel = states.at(i - 1).vel;
        graph.add(
          MotionFactor<wave::PoseVelImuBias, wave::PoseVelImuBias>(i - 1,
                                                                i,
                                                                delta_t,
                                                                model));
    }

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    for (uint64_t i = 0; i < result.size(); i++) {
        PoseVelImuBias res = result.at<PoseVelImuBias>(i);
        EXPECT_TRUE(gtsam::traits<PoseVelImuBias>::Equals(states.at(i),
                                                        res,
                                                        1e-3));
    }
}

}  // namespace wave
