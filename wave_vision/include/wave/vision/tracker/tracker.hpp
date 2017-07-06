/**
 * @file
 * Brute force matcher implementation, derived from descriptor matcher base.
 * @ingroup vision
 */
#ifndef WAVE_VISION_TRACKER_HPP
#define WAVE_VISION_TRACKER_HPP

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

    /** Registers the latest matched keypoints with IDs. Assigns a new ID if one
     * has not already been provided.
     *
     * @param matches the matches between the current and previous images.
     * @return the map corresponding current keypoints to IDs.
     */
    std::map<int, size_t> registerKeypoints(
      const std::vector<cv::DMatch> &matches);

    /** Generates the feature tracks in the current image.
     *
     * Generates the feature tracks in the current image, and also updates the
     * correspondence map between the currently tracked features and their IDs.
     *
     * @param curr_kp the keypoints detected in the current image.
     * @param matches the matches between the current and previous images.
     * @return All feature tracks for the current image
     */
    std::vector<FeatureTrack> generateFeatureTracks(
      const std::vector<cv::KeyPoint> curr_kp,
      const std::vector<cv::DMatch> matches);

    /** Track features within the next image in the sequence.
     *
     * @param next_image next image in the sequence.
     * @return all feature tracks within the image.
     */
    std::vector<FeatureTrack> trackImage(const cv::Mat &next_image);

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
    std::vector<cv::KeyPoint> prev_kp;
    cv::Mat prev_desc;

    std::map<size_t, FeatureTrack> id_map;

    size_t img_count = 0;
};
/** @} group vision */
}  // namespace wave

#include "impl/tracker.hpp"

#endif  // WAVE_VISION_TRACKER_HPP
