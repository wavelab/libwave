#include "wave/wave_test.hpp"
#include "wave/utils/utils.hpp"
#include "wave/vision/dataset.hpp"
#include "wave/optimization/ceres/ba.hpp"

namespace wave {

const std::string TEST_CONFIG = "tests/data/vo_test.yaml";

static std::vector<LandmarkId> build_landmark_ids(
  const std::vector<LandmarkObservation> &features_observed) {
    std::vector<LandmarkId> landmark_ids(features_observed.size());

    for (size_t i = 0; i < features_observed.size(); i++) {
        landmark_ids[i] = features_observed[i].first;
    }

    return landmark_ids;
}

static MatX build_feature_matrix(
  const std::vector<LandmarkObservation> &features_observed) {
    MatX features(features_observed.size(), 2);

    for (size_t i = 0; i < features_observed.size(); i++) {
        features.block(i, 0, 1, 2) = features_observed[i].second.transpose();
    }

    return features;
}

TEST(BAResidual, constructor) {
    // TEST DEFAULT CONSTRUCTOR
    BAResidual residual1;

    EXPECT_FLOAT_EQ(0.0, residual1.fx);
    EXPECT_FLOAT_EQ(0.0, residual1.fy);
    EXPECT_FLOAT_EQ(0.0, residual1.cx);
    EXPECT_FLOAT_EQ(0.0, residual1.cy);
    EXPECT_FLOAT_EQ(0.0, residual1.x);
    EXPECT_FLOAT_EQ(0.0, residual1.y);

    // TEST CONSTRUCTOR WITH PARAMETERS
    Mat3 K;
    // clang-format off
  K << 1.0, 0.0, 3.0,
       0.0, 2.0, 4.0,
       0.0, 0.0, 1.0;
    // clang-format on
    Vec2 x = Vec2{130, 62};

    BAResidual residual2(K, x);
    EXPECT_FLOAT_EQ(K(0, 0), residual2.fx);
    EXPECT_FLOAT_EQ(K(1, 1), residual2.fy);
    EXPECT_FLOAT_EQ(K(0, 2), residual2.cx);
    EXPECT_FLOAT_EQ(K(1, 2), residual2.cy);
    EXPECT_FLOAT_EQ(x(0), residual2.x);
    EXPECT_FLOAT_EQ(x(1), residual2.y);
}

TEST(BAResidual, testResidual) {
    // create vo dataset
    VOTestDatasetGenerator generator;
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();
    LandmarkMap landmarks = dataset.landmarks;

    // translate
    auto true_G_p_B_G = dataset.states[0].robot_G_p_B_G;
    auto t_vec = true_G_p_B_G.data();

    // orientation of robot Body in Global frame
    const auto q_GB = dataset.states[0].robot_q_GB;

    // rotate the body frame by this to get the camera frame
    // or, use this to transform points from camera frame to body frame
    Quaternion q_BC{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                    Eigen::AngleAxisd(0, Vec3::UnitY()) *
                    Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

    Quaternion q_GC = q_GB * q_BC;

    auto q_vec = q_GC.coeffs().data();

    double e[2] = {0.0, 0.0};
    const auto &feature = dataset.states[0].features_observed[0].second;
    const auto &landmark_id = dataset.states[0].features_observed[0].first;

    // test and assert
    BAResidual residual{dataset.camera_K, feature};
    residual(q_vec, t_vec, landmarks[landmark_id].data(), e);
    EXPECT_NEAR(0.0, e[0], 0.0001);
    EXPECT_NEAR(0.0, e[1], 0.0001);
}

TEST(BundleAdjustment, solve) {
    // create vo dataset
    VOTestDatasetGenerator generator;
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();

    // build bundle adjustment problem
    BundleAdjustment ba;

    // We'll keep the parameters in a vector for now.
    // Note we have to pre-allocate the vector, so that pointers to the elements
    // don't change as we insert.
    std::vector<Vec3> params_G_p_C_G(dataset.states.size());
    std::vector<Quaternion> params_q_GC(dataset.states.size());
    LandmarkMap params_landmarks = dataset.landmarks;

    // Add offset to landmark initial estimates
    for (auto &l : params_landmarks) {
        l.second += Vec3{0.01, -0.01, 0.01};
    }

    // Add pose parameters
    for (size_t i = 0; i < dataset.states.size(); i++) {
        // translation
        const auto &true_G_p_C_G = dataset.states[i].robot_G_p_B_G;
        // add offset
        Vec3 G_p_C_G = true_G_p_C_G + Vec3{0.01, 0, -0.005};
        params_G_p_C_G[i] = G_p_C_G;

        // get rotation as quaternion
        const auto &initial_q_GB = dataset.states[i].robot_q_GB;

        // add some noise (or at least an offset)
        auto offset = Quaternion{Eigen::AngleAxisd{0.1, Vec3::UnitX()}};
        auto noisy_q_GB = initial_q_GB * offset;


        // Transform from NWU to EDN
        // This involves the rotation sequence: -90 deg about initial x axis,
        // 0, then -90 deg about initial z axis.
        auto q_BC = Quaternion{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                               Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

        auto q_GC = Quaternion{noisy_q_GB * q_BC};
        params_q_GC[i] = q_GC;

        // add camera
        auto landmark_ids =
          build_landmark_ids(dataset.states[i].features_observed);
        MatX features =
          build_feature_matrix(dataset.states[i].features_observed);
        ba.addCamera(dataset.camera_K,
                     features,
                     landmark_ids,
                     params_G_p_C_G[i].data(),
                     params_q_GC[i].coeffs().data(),
                     params_landmarks);

        // Set a prior on first pose
        if (i == 0) {
            params_G_p_C_G[i] = true_G_p_C_G;
            params_q_GC[i] = initial_q_GB * q_BC;
            ba.problem.SetParameterBlockConstant(params_G_p_C_G[0].data());
            ba.problem.SetParameterBlockConstant(
              params_q_GC[0].coeffs().data());
        }
    }

    // test
    ba.solve();

    for (auto i = 0u; i < dataset.states.size(); i++) {
        // Note: we gave the camera pose, not the robot pose, to the optimizer
        Quaternion q_BC{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                        Eigen::AngleAxisd(0, Vec3::UnitY()) *
                        Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

        const auto true_q_GC = Quaternion{dataset.states[i].robot_q_GB * q_BC};
        const auto &true_G_p_C_G = dataset.states[i].robot_G_p_B_G;
        const auto angular_dist = true_q_GC.angularDistance(params_q_GC[i]);
        const auto linear_dist = (true_G_p_C_G - params_G_p_C_G[i]).norm();

        EXPECT_LT(angular_dist, 1e-6);
        // For some reason the robot position estimate is not as good as the
        // other variables (@todo)
        EXPECT_LT(linear_dist, 0.2);
    }

    for (const auto l : dataset.landmarks) {
        const auto &true_landmark = dataset.landmarks.at(l.first);
        const auto &estimated_landmark = params_landmarks.at(l.first);
        const auto linear_dist = (true_landmark - estimated_landmark).norm();
        EXPECT_LT(linear_dist, 1e-5);
    }
}

}  // namespace wave
