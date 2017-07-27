#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/utils/log.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/detector/fast.yaml";

// Checks that the default configuration has no issues
TEST(FASTTests, GoodInitialization) {
    EXPECT_NO_THROW(FASTDetector detector);
}

// Checks that incorrect configuration path throws an exception
TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetectorParams bad_config(bad_path),
                 std::invalid_argument);
}

// Checks that invalid threshold value throws the proper exception
TEST(FASTTests, BadThresholdConfiguration) {
    FASTDetectorParams bad_threshold_config;
    bad_threshold_config.threshold = -1;

    ASSERT_THROW(FASTDetector detector(bad_threshold_config),
                 std::invalid_argument);
}

// Checks that invalid type values throw the proper exception
TEST(FASTTests, BadTypeConfiguration) {
    FASTDetector detector;

    FASTDetectorParams bad_type_config_neg{10, true, -1};
    FASTDetectorParams bad_type_config_pos{10, true, 4};

    ASSERT_THROW(detector.configure(bad_type_config_neg),
                 std::invalid_argument);

    ASSERT_THROW(detector.configure(bad_type_config_pos),
                 std::invalid_argument);
}

TEST(FASTTests, GoodCustomConfig) {
    FASTDetectorParams config;

    FASTDetector detector(config);
}

// Checks that correct configuration values can be set in detector, and also
// read from getConfiguration function.
TEST(FASTTests, GoodPathConfiguration) {
    FASTDetectorParams config(TEST_CONFIG);
    FASTDetector detector(config);

    FASTDetectorParams input_config{10, true, 2};
    FASTDetectorParams output_config;

    // Configure detector with valid values
    detector.configure(input_config);

    // Extract configuration from detector, assert values have been properly set
    output_config = detector.getConfiguration();

    ASSERT_EQ(output_config.threshold, input_config.threshold);
    ASSERT_EQ(output_config.nonmax_suppression,
              input_config.nonmax_suppression);
    ASSERT_EQ(output_config.type, input_config.type);
}
}  // namespace wave
