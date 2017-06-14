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
    ASSERT_EQ(check_config.ratio_rejection, config.ratio_rejection);
    ASSERT_EQ(check_config.ratio_test_heuristic, config.ratio_test_heuristic);
    ASSERT_EQ(check_config.rejection_heuristic, config.rejection_heuristic);
}

TEST(BFTests, CustomParamsConstructorTest) {
    int norm_type = cv::NORM_L2;
    bool cross_check = true;
    bool ratio_rejection = true;
    double ratio_test_heuristic = 0.8;
    int rejection_heuristic = 5;


    // Place defined values into config struct and create BruteForceMatcher
    BFMatcherParams custom_config(norm_type,
                                  cross_check,
                                  ratio_rejection,
                                  ratio_test_heuristic,
                                  rejection_heuristic);

    BruteForceMatcher bfmatcher(custom_config);

    BFMatcherParams config = bfmatcher.getConfiguration();

    ASSERT_EQ(custom_config.norm_type, config.norm_type);
    ASSERT_EQ(cross_check, config.cross_check);
    ASSERT_EQ(ratio_rejection, config.ratio_rejection);
    ASSERT_EQ(ratio_test_heuristic, config.ratio_test_heuristic);
    ASSERT_EQ(rejection_heuristic, config.rejection_heuristic);
}

TEST(BFTests, DISABLED_CustomYamlConstructorTest) {
    int norm_type = cv::NORM_HAMMING;
    bool cross_check = true;
    bool ratio_rejection = true;
    double ratio_test_heuristic = 0.8;
    int rejection_heuristic = 5;

    BruteForceMatcher bfmatcher(TEST_CONFIG);

    BFMatcherParams config = bfmatcher.getConfiguration();

    std::cout << config.ratio_test_heuristic << std::endl;

    ASSERT_EQ(norm_type, config.norm_type);
    ASSERT_EQ(cross_check, config.cross_check);
    ASSERT_EQ(ratio_rejection, config.ratio_rejection);
    ASSERT_EQ(ratio_test_heuristic, config.ratio_test_heuristic);
    ASSERT_EQ(rejection_heuristic, config.rejection_heuristic);
}

TEST(BFTests, BadNormType) {
    int bad_norm_type_neg = -1;
    int bad_norm_type_high = 8;
    int bad_norm_type_nd = 3;
    bool ratio_rejection = true;
    double ratio_test_heuristic = 0.8;
    int rejection_heuristic = 5;

    bool cross_check = false;

    auto config_neg = BFMatcherParams(bad_norm_type_neg,
                                      cross_check,
                                      ratio_rejection,
                                      ratio_test_heuristic,
                                      rejection_heuristic);
    auto config_high = BFMatcherParams(bad_norm_type_high,
                                       cross_check,
                                       ratio_rejection,
                                       ratio_test_heuristic,
                                       rejection_heuristic);
    auto config_nd = BFMatcherParams(bad_norm_type_nd,
                                     cross_check,
                                     ratio_rejection,
                                     ratio_test_heuristic,
                                     rejection_heuristic);

    ASSERT_THROW(BruteForceMatcher bfmatcher(config_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_high),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher(config_nd), std::invalid_argument);
}

TEST(BFTests, BadRatioTestHeuristic) {
    BFMatcherParams bad_rth_neg;
    BFMatcherParams bad_rth_high;

    bad_rth_neg.ratio_test_heuristic = -0.5;
    bad_rth_high.ratio_test_heuristic = 1.5;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rth_neg),
                 std::invalid_argument);
    ASSERT_THROW(BruteForceMatcher bfmatcher_high(bad_rth_high),
                 std::invalid_argument);
}

TEST(BFTests, BadRejectionHeuristic) {
    BFMatcherParams bad_rh_neg;

    bad_rh_neg.rejection_heuristic = -1;

    ASSERT_THROW(BruteForceMatcher bfmatcher_neg(bad_rh_neg),
                 std::invalid_argument);
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

TEST(BFTests, MatchDescriptorsRejection) {
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
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

    good_matches = bfmatcher.removeOutliers(matches);

    cv::drawMatches(image_1,
                    keypoints_1,
                    image_2,
                    keypoints_2,
                    good_matches,
                    img_with_matches);

    cv::imshow("good matches", img_with_matches);

    cv::waitKey(0);
}
}
