#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

const auto TEST_IMAGE_1 = "tests/data/image_center.png";
const auto TEST_IMAGE_2 = "tests/data/image_right.png";
const auto FIRST_IMG_PATH = "tests/data/tracker_test_sequence/frame0057.jpg";

TEST(BFTests, DistanceMatchDescriptors) {
    std::vector<cv::DMatch> matches;
    cv::Mat img_with_matches;

    cv::Mat image_1, image_2;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    FASTDetector fast;
    BRISKDescriptor brisk;

    BFMatcherParams config(
      cv::NORM_HAMMING, false, 0.8, 5, true, cv::FM_RANSAC);

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

    cv::destroyAllWindows();
}

TEST(BFTests, KnnMatchDescriptors) {
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

    cv::destroyAllWindows();
}

TEST(BFTests, 8Point) {
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

    cv::destroyAllWindows();
}

TEST(BFTests, 7Point) {
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

    cv::destroyAllWindows();
}

TEST(BFTests, LMEDS) {
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

    cv::destroyAllWindows();
}

TEST(BFTests, SequenceMatches) {
    std::vector<cv::KeyPoint> prev_kp, curr_kp;
    cv::Mat prev_desc, curr_desc;
    std::vector<cv::DMatch> matches;
    std::vector<cv::Mat> image_sequence;

    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    image_sequence = readImageSequence(FIRST_IMG_PATH);

    std::vector<cv::Mat>::const_iterator prev_it;
    std::vector<cv::Mat>::const_iterator img_it = image_sequence.begin();

    prev_kp = detector.detectFeatures(*img_it);
    prev_desc = descriptor.extractDescriptors(*img_it, prev_kp);
    prev_it = img_it;
    ++img_it;

    for (img_it = img_it; img_it != image_sequence.end(); ++img_it) {
        cv::Mat img_with_matches;

        curr_kp = detector.detectFeatures(*img_it);
        curr_desc = descriptor.extractDescriptors(*img_it, curr_kp);
        matches =
          matcher.matchDescriptors(prev_desc, curr_desc, prev_kp, curr_kp);

        // Test has been confirmed visually
        cv::drawMatches(
          *prev_it, prev_kp, *img_it, curr_kp, matches, img_with_matches);

        cv::imshow("Sequential Matches", img_with_matches);

        cv::waitKey(0);

        prev_kp = curr_kp;
        prev_desc = curr_desc;
        ++prev_it;

        cv::destroyAllWindows();
    }
}
}  // namespace wave
