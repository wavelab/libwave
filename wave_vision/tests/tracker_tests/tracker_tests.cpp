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

TEST(TrackerTests, ConstructorTest) {
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);
}

TEST(TrackerTests, NoImages) {
    std::vector<cv::Mat> image_sequence;
    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    ASSERT_THROW(tracker.offlineTracker(image_sequence), std::length_error);
}

TEST(TrackerTests, OfflineTrackerTest) {
    std::vector<cv::Mat> image_sequence;
    std::vector<std::vector<FeatureTrack>> feature_tracks;
    std::vector<cv::Mat> drawn_images;

    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    image_sequence = readImageSequence(FIRST_IMG_PATH);

    feature_tracks = tracker.offlineTracker(image_sequence);

    drawn_images = tracker.drawFeatureTracks(feature_tracks, image_sequence);

    // for (const auto &img : drawn_images) {
    //     cv::imshow("Feature Tracks", img);

    //     cv::waitKey(0);
    // }

    ASSERT_NE((int) feature_tracks.size(), 0);
}
}  // namespace wave
