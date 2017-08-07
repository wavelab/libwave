#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/descriptor/orb_descriptor.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/detector/orb_detector.hpp"

namespace wave {

const auto TEST_IMAGE = "tests/data/image_center.png";

TEST(BRISKTests, ComputeFASTBRISKDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    cv::Mat image = cv::imread(TEST_IMAGE);
    std::vector<cv::KeyPoint> keypoints;

    FASTDetector fast;
    BRISKDescriptor brisk;

    keypoints = fast.detectFeatures(image);
    descriptors = brisk.extractDescriptors(image, keypoints);

    ASSERT_FALSE(descriptors.empty());

    // Visual test to verify descriptors are being computed properly
    cv::drawKeypoints(descriptors, keypoints, image_with_keypoints);
    cv::imshow("BRISK Descriptors", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

TEST(BRISKTESTS, ComputeORBBRISKDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    cv::Mat image = cv::imread(TEST_IMAGE);
    std::vector<cv::KeyPoint> keypoints;

    ORBDetector orb;
    BRISKDescriptor brisk;

    keypoints = orb.detectFeatures(image);
    descriptors = brisk.extractDescriptors(image, keypoints);

    ASSERT_FALSE(descriptors.empty());

    // Visual test to verify descriptors are being computed properly
    cv::drawKeypoints(descriptors, keypoints, image_with_keypoints);
    cv::imshow("BRISK Descriptors", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

TEST(ORBDescriptorTests, ComputeORBORBDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    cv::Mat image = cv::imread(TEST_IMAGE);
    std::vector<cv::KeyPoint> keypoints;

    ORBDetector detector;
    ORBDescriptor descriptor;

    keypoints = detector.detectFeatures(image);
    descriptors = descriptor.extractDescriptors(image, keypoints);

    ASSERT_FALSE(descriptors.empty());

    // Visual test to verify descriptors are being computed properly
    cv::drawKeypoints(descriptors, keypoints, image_with_keypoints);
    cv::imshow("ORB Descriptors", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

TEST(FASTORBDescriptorTests, ComputeFASTORBDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    cv::Mat image = cv::imread(TEST_IMAGE);
    std::vector<cv::KeyPoint> keypoints;

    FASTDetector detector;
    ORBDescriptor descriptor;

    keypoints = detector.detectFeatures(image);
    descriptors = descriptor.extractDescriptors(image, keypoints);

    ASSERT_FALSE(descriptors.empty());

    // Visual test to verify descriptors are being computed properly
    cv::drawKeypoints(descriptors, keypoints, image_with_keypoints);
    cv::imshow("ORB Descriptors", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}
}  // namespace wave
