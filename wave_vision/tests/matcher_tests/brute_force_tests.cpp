// Libwave Headers
#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/matcher/brute_force_matcher.yaml";
const auto TEST_IMAGE_1 = "tests/data/image_center.png";
const auto TEST_IMAGE_2 = "tests/data/image_right.png";

// Checks that correct configuration can be loaded
TEST(BFTests, GoodInitialization) {
    ASSERT_NO_THROW(BruteForceMatcher bfmatcher(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(BFTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BruteForceMatcher bfmatcher(bad_path), std::invalid_argument);
}

TEST(BFTests, DefaultConstructorTest) {
    BruteForceMatcher bfmatcher;

    auto check_config = BFMatcherParams{};

    auto config = bfmatcher.getConfiguration();

    ASSERT_EQ(check_config.norm_type, config.norm_type);
    ASSERT_EQ(check_config.cross_check, config.cross_check);
}

TEST(BFTests, CustomParamsConstructorTest) {
    int norm_type = cv::NORM_L2;
    bool cross_check = true;

    // Place defined values into config struct and create BruteForceMatcher
    auto custom_config = BFMatcherParams(norm_type, cross_check);

    BruteForceMatcher bfmatcher(custom_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(custom_config.norm_type, config.norm_type);
    ASSERT_EQ(cross_check, config.cross_check);
}

TEST(BFTests, CustomYamlConstructorTest) {
    int norm_type = cv::NORM_HAMMING;
    bool cross_check = false;

    BruteForceMatcher bfmatcher(TEST_CONFIG);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(norm_type, config.norm_type);
    ASSERT_EQ(cross_check, config.cross_check);
}

TEST(BFTests, BadNormType) {
    int bad_norm_type_neg = -1;
    int bad_norm_type_high = 8;
    int bad_norm_type_nd = 3;

    bool cross_check = false;

    auto config_neg = BFMatcherParams(bad_norm_type_neg, cross_check);
    auto config_high = BFMatcherParams(bad_norm_type_high, cross_check);
    auto config_nd = BFMatcherParams(bad_norm_type_nd, cross_check);

    ASSERT_THROW(BruteForceMatcher bfmatcher(config_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_high),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_nd), std::invalid_argument);
}

TEST(BFTests, DISABLED_MatchDescriptors) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;
    BruteForceMatcher bfmatcher;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Match descriptors from image 1 and image 2
    matches = bfmatcher.matchDescriptors(descriptors_1, descriptors_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("matches", img_with_matches);

    cv::waitKey(0);
}
}  // namespace wave
