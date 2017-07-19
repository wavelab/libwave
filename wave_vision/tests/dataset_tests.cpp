#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"

namespace wave {

const std::string TEST_CONFIG = "tests/data/vo_test.yaml";
const std::string TEST_OUTPUT = "/tmp/dataset_test";

TEST(VOTestCamera, constructor) {
    VOTestCamera camera;

    EXPECT_EQ(0, camera.image_width);
    EXPECT_EQ(0, camera.image_height);
    EXPECT_TRUE(camera.K.isApprox(Mat3::Identity()));
    EXPECT_EQ(0, camera.hz);

    EXPECT_EQ(0, camera.dt);
    EXPECT_EQ(0, camera.frame);
}

TEST(VOTestCamera, update) {
    VOTestCamera camera;
    bool retval;

    // setup
    camera.hz = 100;

    // test
    retval = camera.update(0.1);
    EXPECT_TRUE(retval);
    EXPECT_EQ(1, camera.frame);
    EXPECT_FLOAT_EQ(0.0, camera.dt);
}

TEST(VOTestCamera, observeLandmarks) {
    // setup
    Vec3 G_p_GC{0, 0, 0};

    // Orientation of camera in Global frame: Z looks along X axis
    Quaternion q_GC = Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                      Eigen::AngleAxisd(0, Vec3::UnitY()) *
                      Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX());

    LandmarkMap landmarks;
    landmarks.emplace(1, Vec3{3.0, 0, 0});        // directly in front
    landmarks.emplace(2, Vec3{-3.0, 1, 3});       // behind
    landmarks.emplace(3, Vec3{5.0, 2.2, 3.3});    // above and to the left
    landmarks.emplace(4, Vec3{1.0, 223.0, -19});  // in front, but far off

    VOTestCamera camera;
    camera.hz = 60;
    camera.image_width = 640;
    camera.image_height = 640;
    camera.K << 554.38, 0.0, 320,  //
      0.0, 554.38, 320,            //
      0.0, 0.0, 1.0;

    std::vector<LandmarkObservation> observed;
    auto res = camera.observeLandmarks(0.1, landmarks, q_GC, G_p_GC, observed);
    EXPECT_EQ(0, res);
    ASSERT_EQ(2u, observed.size());  // expect only visible landmarks
    // Landmark 1, directly in front
    EXPECT_EQ(1u, observed[0].first);
    EXPECT_PRED2(VectorsNear, Vec2(320, 320), observed[0].second);
    // Landmark 3, above and to the left
    EXPECT_EQ(3u, observed[1].first);
    EXPECT_LT(observed[1].second.x(), 320);
    EXPECT_LT(observed[1].second.y(), 320);
}

TEST(VOTestDataset, constructor) {
    VOTestDatasetGenerator dataset;

    EXPECT_EQ(0, dataset.camera.image_width);
    EXPECT_EQ(0, dataset.camera.image_height);
}

TEST(VOTestDataset, configure) {
    VOTestDatasetGenerator dataset;

    EXPECT_NO_THROW(dataset.configure(TEST_CONFIG));

    EXPECT_EQ(640, dataset.camera.image_width);
    EXPECT_EQ(640, dataset.camera.image_height);
    EXPECT_FLOAT_EQ(554.25, dataset.camera.K(0, 0));
    EXPECT_FLOAT_EQ(554.25, dataset.camera.K(1, 1));
    EXPECT_FLOAT_EQ(0.0, dataset.camera.K(2, 0));
    EXPECT_FLOAT_EQ(0.0, dataset.camera.K(2, 1));
}

TEST(VOTestDataset, generate) {
    VOTestDatasetGenerator generator;

    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();
}

TEST(VOTestDataset, writeAndReadToFile) {
    // To test both operations, we write to files, read them, and ensure the
    // resulting dataset is the one we started with
    const auto tol = 1e-5;  // Required precision for Eigen's isApprox

    VOTestDatasetGenerator generator;

    remove_dir(TEST_OUTPUT);
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();

    // Write
    dataset.outputToDirectory(TEST_OUTPUT);

    // Read
    auto input = VOTestDataset::loadFromDirectory(TEST_OUTPUT);

    EXPECT_EQ(dataset.camera_K, input.camera_K);
    for (const auto &l : dataset.landmarks) {
        const auto id = l.first;
        ASSERT_TRUE(input.landmarks.count(id));
        EXPECT_PRED3(VectorsNearPrec, l.second, input.landmarks[id], tol);
    }

    // Check each state
    ASSERT_EQ(dataset.states.size(), input.states.size());
    for (auto i = 0u; i < dataset.states.size(); ++i) {
        const auto &lhs = dataset.states[i];
        const auto &rhs = input.states[i];
        EXPECT_DOUBLE_EQ(lhs.time, rhs.time);
        // EXPECT_EQ(lhs.camera_frame, rhs.camera_frame); // Note not checked
        EXPECT_PRED3(VectorsNearPrec, lhs.robot_G_p_GB, rhs.robot_G_p_GB, tol);
        EXPECT_PRED3(VectorsNearPrec,
                     lhs.robot_q_GB.coeffs(),
                     rhs.robot_q_GB.coeffs(),
                     tol);
        ASSERT_EQ(lhs.features_observed.size(), rhs.features_observed.size());

        // Check each feature observation
        for (auto k = 0u; k < lhs.features_observed.size(); ++k) {
            const auto &lhs_feature = lhs.features_observed[k];
            const auto &rhs_feature = rhs.features_observed[k];
            EXPECT_EQ(lhs_feature.first, rhs_feature.first);
            EXPECT_PRED3(
              VectorsNearPrec, lhs_feature.second, rhs_feature.second, tol);
        }
    }
}

}  // namespace wave
