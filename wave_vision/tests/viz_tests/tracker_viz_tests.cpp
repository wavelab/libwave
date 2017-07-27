#include <chrono>

#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/detector/fast_detector.hpp"
#include "wave/vision/detector/orb_detector.hpp"
#include "wave/vision/descriptor/brisk_descriptor.hpp"
#include "wave/vision/descriptor/orb_descriptor.hpp"
#include "wave/vision/matcher/brute_force_matcher.hpp"
#include "wave/vision/tracker/tracker.hpp"

namespace wave {

const auto FIRST_IMG_PATH = "tests/data/tracker_test_sequence/frame0057.jpg";
const auto KITTI_IMG_PATH = "tests/data/kitti_test_sequence/000000.png";

TEST(TrackerTests, OfflineTrackerTest) {
    std::vector<cv::Mat> image_sequence;
    std::vector<cv::Mat> drawn_images;

    std::vector<std::vector<FeatureTrack>> feature_tracks;

    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    image_sequence = readImageSequence(FIRST_IMG_PATH);

    feature_tracks = tracker.offlineTracker(image_sequence);
    ASSERT_NE((int) feature_tracks.size(), 0);

    size_t img_count = 0;

    for (auto img_seq_it = image_sequence.begin();
         img_seq_it != image_sequence.end();
         img_seq_it++) {
        drawn_images.push_back(tracker.drawTracks(img_count, *img_seq_it));

        ++img_count;
    }

    for (auto img_seq_it = drawn_images.begin();
         img_seq_it != drawn_images.end();
         img_seq_it++) {
        cv::imshow("Feature Tracks", *img_seq_it);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}

TEST(TrackerTests, ORBOfflineTrackerTest) {
    std::vector<cv::Mat> image_sequence;
    std::vector<cv::Mat> drawn_images;

    std::vector<std::vector<FeatureTrack>> feature_tracks;

    ORBDetector detector;
    ORBDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<ORBDetector, ORBDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    image_sequence = readImageSequence(FIRST_IMG_PATH);

    feature_tracks = tracker.offlineTracker(image_sequence);
    ASSERT_NE((int) feature_tracks.size(), 0);

    size_t img_count = 0;

    for (auto img_seq_it = image_sequence.begin();
         img_seq_it != image_sequence.end();
         img_seq_it++) {
        drawn_images.push_back(tracker.drawTracks(img_count, *img_seq_it));

        ++img_count;
    }

    for (auto img_seq_it = drawn_images.begin();
         img_seq_it != drawn_images.end();
         img_seq_it++) {
        cv::imshow("Feature Tracks", *img_seq_it);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}

TEST(TrackerTests, DISABLED_KITTIOfflineTrackerTest) {
    std::vector<cv::Mat> image_sequence;
    std::vector<cv::Mat> drawn_images;

    std::vector<std::vector<FeatureTrack>> feature_tracks;

    FASTDetector detector;
    BRISKDescriptor descriptor;
    BruteForceMatcher matcher;

    Tracker<FASTDetector, BRISKDescriptor, BruteForceMatcher> tracker(
      detector, descriptor, matcher);

    image_sequence = readImageSequence(KITTI_IMG_PATH);

    feature_tracks = tracker.offlineTracker(image_sequence);
    ASSERT_NE((int) feature_tracks.size(), 0);

    size_t img_count = 0;

    for (auto img_seq_it = image_sequence.begin();
         img_seq_it != image_sequence.end();
         img_seq_it++) {
        drawn_images.push_back(tracker.drawTracks(img_count, *img_seq_it));

        ++img_count;
    }

    for (auto img_seq_it = drawn_images.begin();
         img_seq_it != drawn_images.end();
         img_seq_it++) {
        cv::imshow("Feature Tracks", *img_seq_it);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}
}  // namespace wave
