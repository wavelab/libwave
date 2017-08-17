/**
 * @file
 * Feature tracker implementation.
 * @ingroup vision
 */
#ifndef WAVE_VISION_TRACKER_HPP
#define WAVE_VISION_TRACKER_HPP

#include <chrono>
#include <string>
#include <vector>

#include "wave/containers/landmark_measurement.hpp"
#include "wave/containers/landmark_measurement_container.hpp"
#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

using FeatureTrack = std::vector<LandmarkMeasurement<int>>;

/** Image tracker class.
 *
 * The Tracker class is templated on a feature detector, descriptor, and matcher
 * to track features over a sequence of images.
 *
 * @tparam TDetector detector object (FAST, ORB, etc...)
 * @tparam TDescriptor descriptor object (BRISK, ORB, etc...)
 * @tparam TMatcher (BruteForceMatcher, FLANN)
 */
template <typename TDetector, typename TDescriptor, typename TMatcher>
class Tracker {
 public:
    /** Default constructor
     *
     * @param detector detector object (FAST, ORB, etc...)
     * @param descriptor descriptor object (BRISK, ORB, etc...)
     * @param matcher matcher object (BruteForceMatcher, FLANN)
     */
    Tracker(TDetector detector,
            TDescriptor descriptor,
            TMatcher matcher,
            int window_size = 0)
        : detector(detector),
          descriptor(descriptor),
          matcher(matcher),
          window_size(window_size) {
        if (this->window_size < 0) {
            throw std::invalid_argument("window_size cannot be negative!");
        }
    }

    ~Tracker() = default;

    /** Get the tracks of all features in the requested image from the sequence.
     *
     * @param img_num the number of the image to obtain tracks from
     * @return tracks corresponding to all detected landmarks in the image, from
     * the start of time to the given image.
     */
    std::vector<FeatureTrack> getTracks(const int img_num) const;

    /** Track features within an image (presumably the next in a sequence).
     *
     * @param image the image to add.
     * @param current_time the time at which the image was captured
     */
    void addImage(const cv::Mat &image,
                  const std::chrono::steady_clock::time_point &current_time);

    /** Draw tracks for the requested image.
     *
     * @param img_num the number of the image within the sequence
     * @param image the image to draw the tracks on
     * @return the image with the tracks illustrated as arrows.
     */
    cv::Mat drawTracks(const std::vector<FeatureTrack> &feature_tracks,
                       const cv::Mat &image) const;

    /** Offline feature tracking, using list of images already loaded.
     *
     * @param image_sequence the sequence of images to analyze.
     * @return the vector of FeatureTracks in each image.
     */
    std::vector<std::vector<FeatureTrack>> offlineTracker(
      const std::vector<cv::Mat> &image_sequence);

    /** The templated FeatureDetector */
    TDetector detector;

    /** The templated DescriptorExtractor */
    TDescriptor descriptor;

    /** The templated DescriptorMatcher */
    TMatcher matcher;

    /** The size of the LandmarkMeasurementContainer */
    size_t lmc_size = 0;

 private:
    /** For online, real-time tracker operation. Maintains memory by clearing
     *  out values from the measurement container that are outside of this time
     *  window.
     *
     *  If set to zero (default), all measurements are kept for offline use.
     */
    int window_size;

    /** If in online mode, this represents the highest image number that can be
     *  requested to extract tracks from.
     */
    int cleared_img_threshold = 0;

    // Keypoints and descriptors from the previous timestep
    std::vector<cv::KeyPoint> prev_kp;
    cv::Mat prev_desc;

    // Correspondence maps
    std::map<int, size_t> prev_ids;
    std::map<size_t, std::chrono::steady_clock::time_point> img_times;

    // Measurement container variables
    LandmarkMeasurementContainer<LandmarkMeasurement<int>> landmarks;

    // The sensor ID. TODO: Expand this for use with multiple cams.
    int sensor_id = 0;

    /** Generate a new ID for each newly detected feature.
     *
     * @return the assigned ID.
     */
    size_t generateFeatureID() const {
        static size_t id = 0;
        return id++;
    }

    /** Detect features and compute descriptors.
     *
     * Detects features and computes descriptors using the templated detector
     * and descriptor.
     *
     * @param image
     * @param keypoints
     * @param descriptor
     */
    void detectAndCompute(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptor);

    /** Register the current time with the current img_count
     *
     * @param current_time the time at which this image was received
     */
    void timestampImage(
      const std::chrono::steady_clock::time_point &current_time) {
        auto img_count = this->img_times.size();
        this->img_times[img_count] = current_time;
    }

    /** Cleans out the LandmarkMeasurementContainer for images outside the
     *  requested window_size.
     *
     *  @param img the image to remove landmark information for.
     */
    void purgeContainer(const int img);

    /** Registers the latest matched keypoints with IDs. Assigns a new ID if one
     * has not already been provided.
     *
     * @param curr_kp the keypoints detected in the current image.
     * @param matches the matches between the current and previous images.
     * @return the map corresponding current keypoints to IDs.
     */
    std::map<int, size_t> registerKeypoints(
      const std::vector<cv::KeyPoint> &curr_kp,
      const std::vector<cv::DMatch> &matches);
};

/** @} group vision */
}  // namespace wave

#include "impl/tracker.hpp"

#endif  // WAVE_VISION_TRACKER_HPP
