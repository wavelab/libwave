// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/utils/log.hpp"

#include <chrono>

namespace wave {

const auto TEST_CONFIG = "tests/config/detector/fast.yaml";
const auto TEST_IMAGE = "tests/data/image_center.png";

// Checks that correct configuration can be loaded
TEST(FASTTests, GoodInitialization) {
    EXPECT_NO_THROW(FASTDetector detector);
}

// Checks that incorrect configuration path throws an exception
TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetector detector(bad_path), std::invalid_argument);
}

// Checks that invalid threshold value throws the proper exception
TEST(FASTTests, BadThresholdConfiguration) {
    FASTParams bad_threshold_config;
    bad_threshold_config.threshold = -1;

    ASSERT_THROW(FASTDetector detector(bad_threshold_config),
                 std::invalid_argument);
}

// Checks that invalid type values throw the proper exception
TEST(FASTTests, BadTypeConfiguration) {
    FASTDetector detector;

    FASTParams bad_type_config_neg{10, true, -1};
    FASTParams bad_type_config_pos{10, true, 4};

    ASSERT_THROW(detector.configure(bad_type_config_neg),
                 std::invalid_argument);

    ASSERT_THROW(detector.configure(bad_type_config_pos),
                 std::invalid_argument);
}

TEST(FASTTests, GoodCustomConfig) {
    FASTParams config;

    FASTDetector detector(config);
}

// Checks that correct configuration values can be set in detector, and also
// read from getConfiguration function.
TEST(FASTTests, GoodPathConfiguration) {
    FASTParams config(TEST_CONFIG);
    FASTDetector detector(config);

    FASTParams input_config{10, true, 2};
    FASTParams output_config;

    // Configure detector with valid values
    detector.configure(input_config);

    // Extract configuration from detector, assert values have been properly set
    output_config = detector.getConfiguration();

    ASSERT_EQ(output_config.threshold, input_config.threshold);
    ASSERT_EQ(output_config.nonmax_suppression,
              input_config.nonmax_suppression);
    ASSERT_EQ(output_config.type, input_config.type);
}

// Confirms that keypoints can be determined, and image with keypoints can be
// displayed.
TEST(FASTTests, DISABLED_DetectImage) {
    FASTDetector detector;

    cv::Mat image = cv::imread(TEST_IMAGE);

    std::vector<cv::KeyPoint> keypoints = detector.detectFeatures(image);

    ASSERT_NE(keypoints.size(), 0u);

    cv::imshow("Scene", image);

    // Visual test, to confirm that images are displayed properly
    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image,
                      keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Detection Test", image_with_keypoints);

    cv::waitKey(0);
}
}  // namespace wave
