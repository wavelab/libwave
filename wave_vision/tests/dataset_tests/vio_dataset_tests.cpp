#include <boost/filesystem.hpp>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset/VioDataset.hpp"

namespace wave {

const auto test_config_file = "tests/data/vo_test.yaml";
const auto test_output_dir = "/tmp/dataset_test";

TEST(VioDataset, constructor) {
    VioDataset dataset{};
}

}  // wave namespace
