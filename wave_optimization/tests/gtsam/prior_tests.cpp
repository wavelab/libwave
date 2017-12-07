#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "wave/optimization/gtsam/pose_vel_bias.hpp"
#include "wave/optimization/gtsam/pose_prior.hpp"
#include "wave/optimization/gtsam/twist_prior.hpp"
#include "wave/optimization/gtsam/bias_prior.hpp"
#include "wave/wave_test.hpp"

/**
 * Given a single state initialized at identity, will the prior factors pull
 * it to where they say it should be?
 */

namespace wave {

TEST(prior, trivial_problem) {
    PoseVelBias init;

    Eigen::Affine3d T_local_s1;
    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 pose_prior(T_local_s1.matrix());
    decltype(PoseVelBias::vel) vel_prior;
    vel_prior.setRandom();

    decltype(PoseVelBias::bias) bias_prior;
    bias_prior.setRandom();

    PoseVelBias true_state;
    true_state.pose = pose_prior;
    true_state.vel = vel_prior;
    true_state.bias = bias_prior;

    gtsam::Values init_vals;
    init_vals.insert(1, init);
    gtsam::NonlinearFactorGraph graph;

    Mat6 info = Mat6::Identity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);

    auto pose_factor = PosePrior<PoseVelBias>(1, pose_prior, model);
    auto vel_factor = TwistPrior<PoseVelBias>(1, vel_prior, model);

    Mat3 info1 = Mat3::Identity();
    auto model1 = gtsam::noiseModel::Gaussian::Information(info1);

    auto bias_factor = BiasPrior<PoseVelBias>(1, bias_prior, model1);

    graph.add(pose_factor);
    graph.add(vel_factor);
    graph.add(bias_factor);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals);
    auto results = optimizer.optimize();
    auto result = results.at<PoseVelBias>(1);

    EXPECT_TRUE(gtsam::traits<PoseVelBias>::Equals(result, true_state));
}
}
