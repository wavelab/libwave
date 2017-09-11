#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/detector/orb_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/descriptor/orb_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"
#include "wave/vision/matcher/flann_matcher.hpp"

namespace wave {

const auto TEST_IMAGE_1 = "tests/data/image_center.png";
const auto TEST_IMAGE_2 = "tests/data/image_right.png";

TEST(BFTests, DistanceMatchDescriptors) {
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config(cv::NORM_HAMMING, 5, true, cv::FM_RANSAC);
    BruteForceMatcher bfmatcher(config);

    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);

    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Match descriptors from image 1 and image 2
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(BFTests, KnnMatchDescriptors) {
    FASTDetector fast;
    BRISKDescriptor brisk;
    BruteForceMatcher bfmatcher;

    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);

    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(BFTests, 8Point) {
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_8POINT;
    BruteForceMatcher bfmatcher(config);

    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);

    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(BFTests, 7Point) {
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_7POINT;
    BruteForceMatcher bfmatcher(config);

    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);

    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(BFTests, LMEDS) {
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config;
    config.fm_method = cv::FM_LMEDS;
    BruteForceMatcher bfmatcher(config);

    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = fast.detectFeatures(image_1);
    keypoints_2 = fast.detectFeatures(image_2);

    descriptors_1 = brisk.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = brisk.extractDescriptors(image_2, keypoints_2);

    // Use KNN Matcher to match
    matches = bfmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test has been confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(FLANNTests, KDTree) {
    ORBDetector detector;
    ORBDescriptor descriptor;

    // Matcher
    FLANNMatcherParams config;
    config.flann_method = FLANN::KDTree;
    FLANNMatcher flannmatcher(config);

    // OpenCV vars
    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = detector.detectFeatures(image_1);
    keypoints_2 = detector.detectFeatures(image_2);

    descriptors_1 = descriptor.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = descriptor.extractDescriptors(image_2, keypoints_2);

    matches = flannmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(FLANNTests, KMeans) {
    ORBDetector detector;
    ORBDescriptor descriptor;

    // Matcher
    FLANNMatcherParams config;
    config.flann_method = FLANN::KMeans;
    FLANNMatcher flannmatcher(config);

    // OpenCV vars
    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = detector.detectFeatures(image_1);
    keypoints_2 = detector.detectFeatures(image_2);

    descriptors_1 = descriptor.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = descriptor.extractDescriptors(image_2, keypoints_2);

    matches = flannmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(FLANNTests, Composite) {
    ORBDetector detector;
    ORBDescriptor descriptor;

    // Matcher
    FLANNMatcherParams config;
    config.flann_method = FLANN::Composite;
    FLANNMatcher flannmatcher(config);

    // OpenCV vars
    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = detector.detectFeatures(image_1);
    keypoints_2 = detector.detectFeatures(image_2);

    descriptors_1 = descriptor.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = descriptor.extractDescriptors(image_2, keypoints_2);

    matches = flannmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

TEST(FLANNTests, LSH) {
    ORBDetector detector;
    ORBDescriptor descriptor;

    // Matcher
    FLANNMatcherParams config;
    config.flann_method = FLANN::LSH;
    FLANNMatcher flannmatcher(config);

    // OpenCV vars
    cv::Mat image_1, image_2, img_with_matches;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::DMatch> matches;

    image_1 = cv::imread(TEST_IMAGE_1, cv::IMREAD_COLOR);
    image_2 = cv::imread(TEST_IMAGE_2, cv::IMREAD_COLOR);

    keypoints_1 = detector.detectFeatures(image_1);
    keypoints_2 = detector.detectFeatures(image_2);

    descriptors_1 = descriptor.extractDescriptors(image_1, keypoints_1);
    descriptors_2 = descriptor.extractDescriptors(image_2, keypoints_2);

    matches = flannmatcher.matchDescriptors(
      descriptors_1, descriptors_2, keypoints_1, keypoints_2);

    // Test confirmed visually
    cv::drawMatches(
      image_1, keypoints_1, image_2, keypoints_2, matches, img_with_matches);

    cv::imshow("good_matches", img_with_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();
}
}  // namespace wave
