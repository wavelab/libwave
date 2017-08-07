#include <boost/filesystem.hpp>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset/VioDatasetGenerator.hpp"

namespace wave {

const auto test_config_file = "tests/data/vo_test.yaml";

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
    const auto tol = 1e-5;  // Required precision for Eigen's isApprox

    VoDatasetGenerator generator;

    remove_dir(test_output_dir);
    generator.configure(test_config_file);
    auto dataset = generator.generate();

    // Write
    dataset.outputToDirectory(test_output_dir);

    // Read
    auto input = VoDataset::loadFromDirectory(test_output_dir);

    EXPECT_EQ(dataset.camera_K, input.camera_K);
    for (const auto &l : dataset.landmarks) {
        const auto id = l.first;
        ASSERT_TRUE(input.landmarks.count(id));
        EXPECT_PRED3(VectorsNearPrec, l.second, input.landmarks[id], tol);
    }
}

}  // wave namespace
