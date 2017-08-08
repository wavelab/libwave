#include <boost/filesystem.hpp>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset/VioDatasetGenerator.hpp"

namespace wave {

const auto test_config_file = "tests/data/vo_test.yaml";
const auto test_output_dir = "/tmp/viodataset_test";


TEST(VioDataset, constructor) {
    VioDataset dataset{};
}

TEST(VioDataset, generate) {
    VioDatasetGenerator generator;
    generator.configure(test_config_file);
    auto dataset = generator.generate();

    // expected value from test_config_file
    EXPECT_EQ(100u, dataset.landmarks.size());
    EXPECT_FALSE(dataset.poses.empty());
}

TEST(VioDataset, writeAndReadToFile) {
    // To test both operations, we write to files, read them, and ensure the
    // resulting dataset is the one we started with
    const auto tol = 1e-4;  // Required precision for Eigen's isApprox

    VioDatasetGenerator generator;

    remove_dir(test_output_dir);
    generator.configure(test_config_file);
    auto dataset = generator.generate();

    // Write
    dataset.outputToDirectory(test_output_dir);

    // Read
    auto input = VioDataset::loadFromDirectory(test_output_dir);

    // Compare landmarks
    for (const auto &l : dataset.landmarks) {
        const auto id = l.first;
        ASSERT_TRUE(input.landmarks.count(id));
        EXPECT_PRED3(VectorsNearPrec, l.second, input.landmarks[id], tol);
    }

    // Compare calibration
    EXPECT_PRED2(MatricesNear, dataset.camera.K, input.camera.K);
    EXPECT_PRED2(VectorsNear, dataset.I_p_IC, input.I_p_IC);
    EXPECT_PRED2(MatricesNear,
                 dataset.R_IC.toRotationMatrix(),
                 input.R_IC.toRotationMatrix());

    // Compare poses
    ASSERT_EQ(dataset.poses.size(), input.poses.size());
    for (auto it = dataset.poses.cbegin(), in_it = input.poses.cbegin();
         it != dataset.poses.cend();
         ++it, ++in_it) {
        const auto &a = it->value;
        const auto &b = in_it->value;
        EXPECT_PRED3(VectorsNearPrec, a.G_p_GI, b.G_p_GI, tol);
        EXPECT_PRED2(
          MatricesNear, a.R_GI.toRotationMatrix(), b.R_GI.toRotationMatrix());
    }

    ASSERT_EQ(dataset.imu_measurements.size(), input.imu_measurements.size());
    for (auto it = dataset.imu_measurements.cbegin(),
              in_it = input.imu_measurements.cbegin();
         it != dataset.imu_measurements.cend();
         ++it, ++in_it) {
        const auto &a = it->value;
        const auto &b = in_it->value;
        EXPECT_PRED3(VectorsNearPrec, a.I_vel_GI, b.I_vel_GI, tol);
        EXPECT_PRED3(VectorsNearPrec, a.I_ang_vel_GI, b.I_ang_vel_GI, tol);
    }

    // Compare feature measurements
    ASSERT_EQ(dataset.feature_measurements.size(),
              input.feature_measurements.size());
    for (auto it = dataset.feature_measurements.cbegin(),
              in_it = input.feature_measurements.cbegin();
         it != dataset.feature_measurements.cend();
         ++it, ++in_it) {
        EXPECT_EQ(it->landmark_id, in_it->landmark_id);
        EXPECT_PRED3(VectorsNearPrec, it->value, in_it->value, tol);
    }
}

}  // wave namespace
