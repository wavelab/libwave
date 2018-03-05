#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/detector/orb_detector.hpp"

namespace wave {

const auto TEST_IMAGE = "tests/data/image_center.png";

// Confirms that keypoints can be determined, and image with keypoints can be
// displayed
TEST(FASTTests, DetectImage) {
    FASTDetector detector;

    cv::Mat image = cv::imread(TEST_IMAGE);

    std::vector<cv::KeyPoint> keypoints = detector.detectFeatures(image);

    ASSERT_FALSE(keypoints.empty());

    // Visual test, to confirm that images are displayed properly
    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image,
                      keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("FAST Detection Test", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

TEST(FASTTests, NumFeatures) {
    FASTDetectorParams config;
    config.num_features = 300;
    FASTDetector detector(config);

    cv::Mat image = cv::imread(TEST_IMAGE);

    std::vector<cv::KeyPoint> keypoints = detector.detectFeatures(image);

    ASSERT_FALSE(keypoints.empty());

    // Visual test, to confirm that images are displayed properly
    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image,
                      keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("FAST Detection Test", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

TEST(ORBDetectorTests, DetectImage) {
    ORBDetectorParams config;
    // Increase the number of features
    config.num_features = 5000;

    ORBDetector detector(config);

    cv::Mat image = cv::imread(TEST_IMAGE);

    std::vector<cv::KeyPoint> keypoints = detector.detectFeatures(image);

    ASSERT_FALSE(keypoints.empty());

    // Visual test, to confirm that images are displayed properly
    // Draw Keypoints on image and display
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image,
                      keypoints,
                      image_with_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB Detection Test", image_with_keypoints);

    cv::waitKey(0);

    cv::destroyAllWindows();
}
}  // namespace wave
