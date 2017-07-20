#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/utils/log.hpp"

namespace wave {

const auto TEST_IMAGE = "tests/data/image_center.png";

// Confirms that keypoints can be determined, and image with keypoints can be
// displayed
TEST(FASTTests, DetectImage) {
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
