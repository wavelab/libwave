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

    std::map<Vec3, int, VecComparator> landmarks;
    landmarks.insert({Vec3::Random(), 1});
    landmarks.insert({Vec3::Random(), 2});
    landmarks.insert({Vec3::Random(), 3});
    landmarks.insert({Vec3::Random(), 4});

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
    std::vector<std::pair<Vec2, int>> observed;
    camera.checkLandmarks(0.1, landmarks, rpy, t, observed);
}

TEST(VOTestDataset, constructor) {
    VOTestDataset dataset;

    EXPECT_EQ(0, dataset.camera.image_width);
    EXPECT_EQ(0, dataset.camera.image_height);
}

TEST(VOTestDataset, configure) {
    VOTestDataset dataset;
    int retval;

    retval = dataset.configure(TEST_CONFIG);
    EXPECT_EQ(0, retval);
    EXPECT_EQ(640, dataset.camera.image_width);
    EXPECT_EQ(640, dataset.camera.image_height);
    EXPECT_FLOAT_EQ(554.25, dataset.camera.K(0, 0));
    EXPECT_FLOAT_EQ(554.25, dataset.camera.K(1, 1));
    EXPECT_FLOAT_EQ(0.0, dataset.camera.K(2, 0));
    EXPECT_FLOAT_EQ(0.0, dataset.camera.K(2, 1));
}

TEST(VOTestDataset, simulateVODataset) {
    VOTestDataset dataset;

    dataset.configure(TEST_CONFIG);
    int retval = dataset.simulateVODataset();

    EXPECT_EQ(0, retval);
}

TEST(VOTestDataset, generateTestData) {
    VOTestDataset dataset;

    remove_dir(TEST_OUTPUT);
    dataset.configure(TEST_CONFIG);
    int retval = dataset.generateTestData(TEST_OUTPUT);
    EXPECT_EQ(0, retval);
}

}  // namespace wave
