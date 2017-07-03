#include <chrono>

#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"
#include "wave/vision/tracker/tracker.hpp"

namespace wave {

const auto IMAGES_PATH = "tests/data/tracker_test_sequence/frame0057.jpg";

TEST(TrackerTests, NoImages) {
    std::vector<cv::Mat> image_sequence;
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    // Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
    //   detector, descriptor, matcher);

    // ASSERT_THROW(tracker.offlineTracker(image_sequence), std::length_error);
}

TEST(TrackerTests, OfflineTrackerTest) {
    std::vector<cv::Mat> image_sequence;
}
}  // namespace wave
