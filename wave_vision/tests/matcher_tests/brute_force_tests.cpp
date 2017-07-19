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
    ASSERT_NO_THROW(BruteForceMatcher bfmatcher);
}

// Checks that incorrect configuration path throws an exception
TEST(BFTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(BFMatcherParams config(bad_path), std::invalid_argument);
}

TEST(BFTests, DefaultConstructorTest) {
    BFMatcherParams check_config(TEST_CONFIG);
    BruteForceMatcher bfmatcher(check_config);

    auto config = bfmatcher.getConfiguration();

    ASSERT_EQ(check_config.norm_type, config.norm_type);
    ASSERT_EQ(check_config.use_knn, config.use_knn);
    ASSERT_EQ(check_config.ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(check_config.distance_threshold, config.distance_threshold);
    ASSERT_EQ(check_config.fm_method, config.fm_method);
}

TEST(BFTests, CustomParamsConstructorTest) {
    int norm_type = cv::NORM_L2;
    bool use_knn = false;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    int fm_method = cv::FM_LMEDS;

    // Place defined values into config struct and create BruteForceMatcher
    BFMatcherParams custom_config(
      norm_type, use_knn, ratio_threshold, distance_threshold, fm_method);

    BruteForceMatcher bfmatcher(custom_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(custom_config.norm_type, config.norm_type);
    ASSERT_EQ(custom_config.use_knn, config.use_knn);
    ASSERT_EQ(custom_config.ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(custom_config.distance_threshold, config.distance_threshold);
    ASSERT_EQ(custom_config.fm_method, config.fm_method);
}

TEST(BFTests, CustomYamlConstructorTest) {
    int norm_type = cv::NORM_HAMMING;
    bool use_knn = true;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    int fm_method = cv::FM_RANSAC;

    BFMatcherParams input_config(TEST_CONFIG);
    BruteForceMatcher bfmatcher(input_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(norm_type, config.norm_type);
    ASSERT_EQ(use_knn, config.use_knn);
    ASSERT_EQ(ratio_threshold, config.ratio_threshold);
    ASSERT_EQ(distance_threshold, config.distance_threshold);
    ASSERT_EQ(fm_method, config.fm_method);
}

TEST(BFTests, BadNormType) {
    int bad_norm_type_neg = -1;
    int bad_norm_type_high = 8;
    int bad_norm_type_nd = 3;
    bool use_knn = true;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    int fm_method = cv::FM_RANSAC;

    auto config_neg = BFMatcherParams(bad_norm_type_neg,
                                      use_knn,
                                      ratio_threshold,
                                      distance_threshold,
                                      fm_method);
    auto config_high = BFMatcherParams(bad_norm_type_high,
                                       use_knn,
                                       ratio_threshold,
                                       distance_threshold,
                                       fm_method);
    auto config_nd = BFMatcherParams(bad_norm_type_nd,
                                     use_knn,
                                     ratio_threshold,
                                     distance_threshold,
                                     fm_method);

    ASSERT_THROW(BruteForceMatcher bfmatcher(config_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_high),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_nd), std::invalid_argument);
}

TEST(BFTests, BadRatioTestHeuristic) {
    BFMatcherParams bad_rth_neg;
    BFMatcherParams bad_rth_high;

    bad_rth_neg.ratio_threshold = -0.5;
    bad_rth_high.ratio_threshold = 1.5;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rth_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_high(bad_rth_high),
                 std::invalid_argument);
}

TEST(BFTests, BadRejectionHeuristic) {
    BFMatcherParams bad_rh_neg;

    bad_rh_neg.distance_threshold = -1;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rh_neg),
                 std::invalid_argument);
}

TEST(BFTests, BadFmMethod) {
    BFMatcherParams low_fm_method;
    BFMatcherParams high_fm_method;
    BFMatcherParams three_fm_method;
    BFMatcherParams mid_fm_method;

    low_fm_method.fm_method = -1;
    high_fm_method.fm_method = 9;
    three_fm_method.fm_method = 3;
    mid_fm_method.fm_method = 5;

    ASSERT_THROW(BruteForceMatcher bfmatcher_low(low_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_high(high_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_three(three_fm_method),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_mid(mid_fm_method),
                 std::invalid_argument);
}

TEST(BFTests, DISABLED_DistanceMatchDescriptors) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config(cv::NORM_HAMMING, false, 0.8, 5, cv::FM_RANSAC);

    BruteForceMatcher bfmatcher(config);

    cv::InputArray mask = cv::noArray();

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Match descriptors from image 1 and image 2
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2, mask);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("matches", img_with_matches);

    cv::waitKey(0);
}

TEST(BFTests, DISABLED_KnnMatchDescriptors) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;
    BruteForceMatcher bfmatcher;

    cv::InputArray mask = cv::noArray();

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2, mask);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);

    cv::waitKey(0);
}

TEST(BFTests, DISABLED_8Point) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_8POINT;
    BruteForceMatcher bfmatcher(config);

    cv::InputArray mask = cv::noArray();

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2, mask);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("matches", img_with_matches);

    cv::waitKey(0);
}

TEST(BFTests, DISABLED_7Point) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_7POINT;
    BruteForceMatcher bfmatcher(config);

    cv::InputArray mask = cv::noArray();

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2, mask);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);

    cv::waitKey(0);
}

TEST(BFTests, DISABLED_LMEDS) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_LMEDS;
    BruteForceMatcher bfmatcher(config);

    cv::InputArray mask = cv::noArray();

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);
    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);
    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2, mask);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);

    cv::waitKey(0);
}
}  // namespace wave
