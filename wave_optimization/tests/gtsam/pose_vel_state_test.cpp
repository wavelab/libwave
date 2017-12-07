#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/optimization/gtsam/motion_factor.hpp"
#include "wave/optimization/gtsam/pose_vel_bias.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(pose_vel_bias_state, example) {
    uint64_t from, to;
    double delta_t = 0.1;
    from = 1;
    to = 2;

    Eigen::Matrix<double, 15, 15> info;
    info.setIdentity();
    auto noise = gtsam::noiseModel::Gaussian::Information(info);
    MotionFactor factor(from, to, delta_t, noise);

    PoseVelBias Start, End;
    Start.vel << 0, 0, 0.1, 5, 0, 0;
    End.vel << 0, 0, 0.1, 5, 0, 0;

    End.pose = Start.pose.Retract(delta_t * Start.vel);

    gtsam::Matrix H1, H2;
    Eigen::Matrix<double, 15, 1> err = factor.evaluateError(Start, End, H1, H2);

    EXPECT_TRUE(err.isZero());
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
        graph.add(MotionFactor(i - 1, i, delta_t, model));
    }

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();

    for (uint64_t i = 0; i < result.size(); i++) {
        PoseVelBias res = result.at<PoseVelBias>(i);
        EXPECT_TRUE(
          gtsam::traits<PoseVelBias>::Equals(states.at(i), res, 1e-3));
    }
}

}  // namespace wave
