#include "wave/wave_test.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

const auto TEST_IMAGE = "tests/data/image_center.png";

TEST(BRISKTests, ComputeDescriptors) {
    cv::Mat descriptors;
    cv::Mat image_with_keypoints;

    cv::Mat image = cv::imread(TEST_IMAGE);
    std::vector<cv::KeyPoint> keypoints;

    FASTDetector fast;
    BRISKDescriptor brisk;

    keypoints = fast.detectFeatures(image);
    descriptors = brisk.extractDescriptors(image, keypoints);

    ASSERT_GT(descriptors.total(), 0u);

    // Visual test to verify descriptors are being computed properly
    cv::imshow("Scene", image);
    cv::drawKeypoints(descriptors, keypoints, image_with_keypoints);
    cv::imshow("Descriptors", image_with_keypoints);

    cv::waitKey(0);
}
}  // namespace wave
