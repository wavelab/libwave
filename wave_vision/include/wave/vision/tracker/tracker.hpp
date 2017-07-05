/**
 * @file
 * Brute force matcher implementation, derived from descriptor matcher base.
 * @ingroup vision
 */
#ifndef WAVE_VISION_TRACKER_HPP
#define WAVE_VISION_TRACKER_HPP

#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "wave/vision/utils.hpp"
#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

class FeatureTrack {
 public:
    // Constructor
    FeatureTrack() {}

    // Destructor
    ~FeatureTrack() = default;

    /** Returns size of the measurement vector.
     *
     * @return size of the measurement vector
     */
    size_t size() const {
        return this->measurement.size();
    }

    /** The assigned ID corresponding to this feature */
    size_t id;
    /** The pixel location of the feature in the sequence of images */
    std::vector<cv::Point2f> measurement;
    /** The first image the feature is seen in */
    size_t first_image;
    /** The last image the feature is seen in */
    size_t last_image;
};

template <typename TDetector, typename TDescriptor, typename TMatcher>
class Tracker {
 public:
    /** Default constructor
     *
     * @param detector detector object (FAST, ORB, etc...)
     * @param descriptor descriptor object (BRISK, ORB, etc...)
     * @param matcher matcher object (BruteForceMatcher, FLANN)
     */
    Tracker(TDetector detector, TDescriptor descriptor, TMatcher matcher)
        : detector(detector), descriptor(descriptor), matcher(matcher) {}

    /** Destructor */
    ~Tracker() = default;

    /** Offline feature tracking, using list of images already loaded. */
    std::vector<std::vector<FeatureTrack>> offlineTracker(
      const std::vector<cv::Mat> &image_sequence);

    /** Draws feature tracks from offline tracking.
     *
     * @param feature_tracks vector of all FeatureTracks from offline tracking
     * @param images the original images
     * @return images with drawn feature tracks
     */
    std::vector<cv::Mat> drawFeatureTracks(
      const std::vector<std::vector<FeatureTrack>> &feature_tracks,
      const std::vector<cv::Mat> &images);

    /** Removes expired IDs from the ID map. */
    void removeExpiredIDs();

 private:
    size_t generateFeatureID() {
        static size_t id = 0;
        return id++;
    }

    void detectAndCompute(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptor);

    TDetector detector;
    TDescriptor descriptor;
    TMatcher matcher;

    std::map<int, size_t> prev_ids;
    std::map<size_t, FeatureTrack> id_map;

    size_t img_count;
};
/** @} group vision */
}  // namespace wave

#include "impl/tracker.hpp"

#endif  // WAVE_VISION_TRACKER_HPP
