#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"

#define TEST_CONFIG "tests/data/vision/dataset/vo_test.xml"
#define TEST_OUTPUT "/tmp/dataset_test"

namespace wave {

TEST(TestCamera, constructor) {
  TestCamera camera;

  EXPECT_EQ(-1, camera.image_width);
  EXPECT_EQ(-1, camera.image_height);
  EXPECT_EQ(-1, camera.hz);

  EXPECT_EQ(0, camera.dt);
  EXPECT_EQ(-1, camera.frame);
}

TEST(TestCamera, update) {
  TestCamera camera;
  bool retval;

  // setup
  camera.hz = 100;

  // test
  retval = camera.update(0.1);
  EXPECT_TRUE(retval);
  EXPECT_EQ(0, camera.frame);
  EXPECT_FLOAT_EQ(0.0, camera.dt);
}

TEST(TestCamera, checkFeatures) {
  TestCamera camera;
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

TEST(TestDataset, constructor) {
  TestDataset dataset;

  EXPECT_EQ(-1, dataset.camera.image_width);
  EXPECT_EQ(-1, dataset.camera.image_height);
}

TEST(TestDataset, configure) {
  TestDataset dataset;
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

TEST(TestDataset, generateTestData) {
  TestDataset dataset;
  int retval;

  dataset.configure(TEST_CONFIG);
  retval = dataset.generateTestData(TEST_OUTPUT);

  EXPECT_EQ(0, retval);
}

}  // end of wave namespace
