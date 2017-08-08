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

}  // wave namespace
