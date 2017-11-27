#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/optimization/gtsam/hand_eye.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(hand_eye, init) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    gtsam::Pose3 T_local_s1;
    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_local_s1, model);
}

// Test that the factor produces zero error when transforms form a chain
TEST(hand_eye, zero_error) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    Eigen::Affine3d T_local_s2;
    Eigen::Affine3d T_s1_s2;
    double B_Z = 0;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    T_s1_s2.matrix() << 1.000000000000000, 0, 0, 0.200000000000000, 0,
      0.995004165278026, -0.099833416646828, 0.300000000000000, 0,
      0.099833416646828, 0.995004165278026, -0.100000000000000, 0, 0, 0,
      1.000000000000000;

    T_local_s2.matrix() << 0.936293363584199, -0.251922821203341,
      0.244723577664956, 32.082894852206735, 0.289629477625516,
      0.947957399267153, -0.132255566480303, 2.348549122632335,
      -0.198669330795061, 0.194709171154326, 0.960530497001443,
      2.892102119622983, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());
    gtsam::Pose3 T_loc_2(T_local_s2.matrix());
    gtsam::Pose3 T_s1s2(T_s1_s2.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_loc_1, model);

    auto err = factor.evaluateError(T_loc_2, T_s1s2, B_Z);

    EXPECT_NEAR(err.norm(), 0, 1e-6);
}

// Test that the factor jacobians are good when error is small
TEST(hand_eye, jacobians) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    Eigen::Affine3d T_local_s2;
    Eigen::Affine3d T_s1_s2;
    double B_Z = 0;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
            0.218350663146334, 32.000000000000000, 0.289629477625516,
            0.956425085849232, -0.036957013524625, 2.000000000000000,
            -0.198669330795061, 0.097843395007256, 0.975170327201816,
            3.000000000000000, 0, 0, 0, 1.000000000000000;

    T_s1_s2.matrix() << 1.000000000000000, 0, 0, 0.200000000000000, 0,
            0.995004165278026, -0.099833416646828, 0.300000000000000, 0,
            0.099833416646828, 0.995004165278026, -0.100000000000000, 0, 0, 0,
            1.000000000000000;

    T_local_s2.matrix() << 0.936293363584199, -0.251922821203341,
            0.244723577664956, 32.082894852206735, 0.289629477625516,
            0.947957399267153, -0.132255566480303, 2.348549122632335,
            -0.198669330795061, 0.194709171154326, 0.960530497001443,
            2.892102119622983, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());
    gtsam::Pose3 T_loc_2(T_local_s2.matrix());
    gtsam::Pose3 T_s1s2(T_s1_s2.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_loc_1, model);

    gtsam::Matrix J_loc2, J_s1s2, J_B_Z;
    auto err = factor.evaluateError(T_loc_2, T_s1s2, B_Z, J_loc2, J_s1s2, J_B_Z);

    auto fun = boost::bind(&HandEyeFactor::evaluateError, boost::ref(factor), _1, _2, _3, boost::none, boost::none, boost::none);

    gtsam::Matrix J_loc2num = gtsam::numericalDerivative31<gtsam::Vector, gtsam::Pose3, gtsam::Pose3, double>(fun, T_loc_2, T_s1s2, B_Z, 1e-6);
    gtsam::Matrix J_s1s2num = gtsam::numericalDerivative32<gtsam::Vector, gtsam::Pose3, gtsam::Pose3, double>(fun, T_loc_2, T_s1s2, B_Z, 1e-6);
    gtsam::Matrix J_BZnum;
    double BZe = B_Z + 1e-6;
    auto err2 = factor.evaluateError(T_loc_2, T_s1s2, BZe, boost::none, boost::none, boost::none);
    J_BZnum = 1e6 * (err2 - err);

    EXPECT_NEAR(err.norm(), 0, 1e-6);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            EXPECT_NEAR(J_loc2(i,j) - J_loc2num(i,j), 0, 1e-8);
            EXPECT_NEAR(J_s1s2(i,j) - J_s1s2num(i,j), 0, 1e-8);
        }
        EXPECT_NEAR(J_BZnum(i), J_B_Z(i), 1e-8);
    }
}


// This tests a small hand-eye problem. Given
// Corrupted measurements of T_0_S1, can the true hand eye transform be found
namespace {

double truncate(double val, double min, double max) {
    if (val < min) {
        return min;
    }
    if (val > max) {
        return max;
    }
    return val;
}

}

TEST(HandEye, Sample_Problem) {
    Eigen::Affine3d T_S1_S2;
    T_S1_S2.matrix() << 1.000000000000000, 0, 0, 0.200000000000000, 0,
            1, 0, 0.300000000000000, 0,
            0, 1, -0.100000000000000, 0, 0, 0,
            1.000000000000000;

    gtsam::NonlinearFactorGraph graph;

    std::vector<gtsam::Pose3> true_T_O_S1, true_T_O_S2;
    std::vector<gtsam::Pose3> noisy_T_O_S1;
    std::vector<gtsam::Pose3> fake_odom;
    gtsam::Pose3 true_T_S1_S2(T_S1_S2.matrix());

    gtsam::Values initial;

    //Generate random poses
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<> angles(-1, 1), displacements(-50, 50);
    std::normal_distribution<> noise_ang(0, 0.01);
    std::normal_distribution<> noise_dis(0, 0.01);
    for(unsigned long i = 0; i < 20; i++) {
        double roll = angles(gen);
        double pitch = angles(gen);
        double yaw = angles(gen);

        double x = displacements(gen);
        double y = displacements(gen);
        double z = displacements(gen);

        gtsam::Point3 rand_pt;
        rand_pt.matrix() << x, y, z;

        gtsam::Pose3 rand_pose(gtsam::Rot3::Ypr(yaw, pitch, roll), rand_pt);
        true_T_O_S1.emplace_back(rand_pose);
        true_T_O_S2.emplace_back(true_T_O_S1.at(i) * true_T_S1_S2);

        roll = roll + truncate(noise_ang(gen), -0.02, 0.02);
        pitch = pitch + truncate(noise_ang(gen), -0.02, 0.02);
        yaw = yaw + truncate(noise_ang(gen), -0.02, 0.02);

        x = x + truncate(noise_dis(gen), -0.02, 0.02);
        y = y + truncate(noise_dis(gen), -0.02, 0.02);
        z = z + truncate(noise_dis(gen), -0.02, 0.02);

        gtsam::Point3 noisy_pt;
        noisy_pt.matrix() << x, y, z;
        gtsam::Pose3 noise_pose(gtsam::Rot3::Ypr(yaw, pitch, roll), noisy_pt);
        noisy_T_O_S1.emplace_back(noise_pose);

        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        graph.push_back(HandEyeFactor(i, 200, 300, noisy_T_O_S1.at(i), model));  // T_O_S1
        initial.insert(i, noisy_T_O_S1.at(i));

        if (i > 0) {
            auto betmodel = gtsam::noiseModel::Gaussian::Information(info);
            fake_odom.push_back(true_T_O_S2.at(i - 1).between(true_T_O_S2.at(i)));
            graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, fake_odom.at(i-1), betmodel));
        }
    }
    gtsam::Pose3 init_T_S1_S2;
    initial.insert(200, init_T_S1_S2);

    gtsam::LevenbergMarquardtParams params;
    params.absoluteErrorTol = 1e-18;
    params.setVerbosity("TERMINATION");
    params.maxIterations = 300000;
    gtsam::LevenbergMarquardtOptimizer optimizer(
            graph, initial, params);

//    gtsam::GaussNewtonParams params;
//    params.setVerbosity("TERMINATION");
//    params.absoluteErrorTol = 1e-18;
//    gtsam::GaussNewtonOptimizer optimizer(graph, initial, params);

    auto result = optimizer.optimize();

    for(unsigned long i = 0; i < 20; i++) {
        EXPECT_NEAR((result.at<gtsam::Pose3>(i).matrix() - true_T_O_S2.at(i).matrix()).norm(), 0, 1e-3);
    }
    gtsam::Pose3 opt_T_S1_S2 = result.at<gtsam::Pose3>(200);
    EXPECT_NEAR((opt_T_S1_S2.matrix() - true_T_S1_S2.matrix()).norm(), 0, 1e-3);

}

}
