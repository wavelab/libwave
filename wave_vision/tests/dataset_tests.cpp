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

TEST(VOTestCamera, checkLandmarks) {
    // setup
    Vec3 rpy{0, 0, 0};
    Vec3 t{0, 0, 0};

    LandmarkMap landmarks;
    landmarks.emplace(1, Vec3::Random());
    landmarks.emplace(2, Vec3::Random());
    landmarks.emplace(3, Vec3::Random());
    landmarks.emplace(4, Vec3::Random());

    // clang-format off
		VOTestCamera camera;
		camera.hz = 60;
		camera.image_width = 640;
		camera.image_height = 640;
		camera.K << 554.38, 0.0, 320,
								0.0, 554.38, 320,
								0.0, 0.0, 1.0;
    // clang-format on

    // test
    std::vector<LandmarkObservation> observed;
    camera.checkLandmarks(0.1, landmarks, rpy, t, observed);
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

TEST(VOTestDataset, outputToFile) {
    VOTestDatasetGenerator generator;

    remove_dir(TEST_OUTPUT);
    generator.configure(TEST_CONFIG);
    auto dataset = generator.generate();

    dataset.outputToFile(TEST_OUTPUT);
}

}  // namespace wave
