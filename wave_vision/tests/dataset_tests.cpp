#include <boost/filesystem.hpp>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"

namespace wave {

const std::string TEST_CONFIG = "tests/data/vo_test.yaml";
const std::string TEST_OUTPUT = "/tmp/dataset_test";

TEST(VOTestCamera, constructor) {
    VOTestCamera camera;

    EXPECT_EQ(-1, camera.image_width);
    EXPECT_EQ(-1, camera.image_height);
    EXPECT_EQ(-1, camera.hz);

    EXPECT_EQ(0, camera.dt);
    EXPECT_EQ(-1, camera.frame);
}

TEST(VOTestCamera, update) {
    VOTestCamera camera;
    bool retval;

    // setup
    camera.hz = 100;

    // test
    retval = camera.update(0.1);
    EXPECT_TRUE(retval);
    EXPECT_EQ(0, camera.frame);
    EXPECT_FLOAT_EQ(0.0, camera.dt);
}

TEST(VOTestCamera, checkFeatures) {
    VOTestCamera camera;
    MatX features;
    Vec3 rpy;
    Vec3 t;
    std::vector<std::pair<Vec2, Vec3>> observed;

    // setup
    features.resize(4, 1);
    features.block(0, 0, 4, 1) << 10, 0, 0, 1;
    rpy << 0, 0, 0;
    t << 0, 0, 0;

    // clang-format off
    camera.hz = 60;
    camera.image_width = 640;
    camera.image_height = 640;
    camera.K << 554.38, 0.0, 320,
                0.0, 554.38, 320,
                0.0, 0.0, 1.0;
    // clang-format on

    // test
    camera.checkFeatures(0.1, features, rpy, t, observed);
}

TEST(VOTestDataset, constructor) {
    // test default constructor
    VOTestDataset dataset;

    EXPECT_EQ(-1, dataset.camera.image_width);
    EXPECT_EQ(-1, dataset.camera.image_height);

    // test constructor with config file path as argument
    VOTestDataset dataset2(TEST_CONFIG);

    EXPECT_EQ(640, dataset2.camera.image_width);
    EXPECT_EQ(640, dataset2.camera.image_height);
    EXPECT_FLOAT_EQ(554.25, dataset2.camera.K(0, 0));
    EXPECT_FLOAT_EQ(554.25, dataset2.camera.K(1, 1));
    EXPECT_FLOAT_EQ(0.0, dataset2.camera.K(2, 0));
    EXPECT_FLOAT_EQ(0.0, dataset2.camera.K(2, 1));
    EXPECT_NO_THROW();
}

TEST(VOTestDataset, generateTestData) {
    VOTestDataset dataset(TEST_CONFIG);

    boost::filesystem::remove_all(TEST_OUTPUT);
    dataset.generateTestData(TEST_OUTPUT);
    EXPECT_NO_THROW();
}

}  // wave namespace
