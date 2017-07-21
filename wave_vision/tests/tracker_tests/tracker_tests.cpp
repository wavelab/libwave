#include <chrono>

#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"
#include "wave/vision/tracker/tracker.hpp"

namespace wave {

// Path to first image for image sequence
const auto FIRST_IMG_PATH = "tests/data/tracker_test_sequence/frame0057.jpg";
const auto TEST_IMAGE_0 = "tests/data/tracker_test_sequence/frame0059.jpg";
const auto TEST_IMAGE_1 = "tests/data/tracker_test_sequence/frame0060.jpg";
const auto TEST_IMAGE_2 = "tests/data/tracker_test_sequence/frame0061.jpg";

TEST(TrackerTests, ConstructorTest) {
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);
}

TEST(TrackerTests, AddImageGetTracks) {
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    std::chrono::steady_clock clock;

    cv::Mat image_1 = cv::imread(TEST_IMAGE_0);
    cv::Mat image_2 = cv::imread(TEST_IMAGE_1);
    cv::Mat image_3 = cv::imread(TEST_IMAGE_2);

    tracker.addImage(image_1, clock.now());
    ASSERT_THROW(tracker.getTracks(-1), std::out_of_range);
    ASSERT_THROW(tracker.getTracks(5), std::out_of_range);

    std::vector<FeatureTrack> ft_0 = tracker.getTracks(0);
    ASSERT_EQ((int) ft_0.size(), 0);

    tracker.addImage(image_2, clock.now());
    tracker.addImage(image_3, clock.now());

    std::vector<FeatureTrack> ft_1 = tracker.getTracks(1);
    std::vector<FeatureTrack> ft_2 = tracker.getTracks(2);

    ASSERT_GE((int) ft_1.size(), 0);
    ASSERT_GE((int) ft_2.size(), 0);
}

TEST(TrackerTests, OfflineTrackerNoImages) {
    std::vector<cv::Mat> image_sequence;
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    ASSERT_THROW(tracker.offlineTracker(image_sequence), std::invalid_argument);
}
}  // namespace wave
