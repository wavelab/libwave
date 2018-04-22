/**
 * @file
 * Feature tracker implementation for a stereo camera setup.
 * @ingroup vision
 */

#ifndef WAVE_STEREO_TRACKER_HPP
#define WAVE_STEREO_TRACKER_HPP

#include "wave/containers/landmark_measurement.hpp"
#include "wave/containers/landmark_measurement_container.hpp"
#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

enum Cameras { LEFT, RIGHT };

/// Define StereoFeature as a pair of pixel locations from the two images.
using StereoFeature = std::pair<Vec2, Vec2>;

struct StereoFeatureTrack {
    std::vector<LandmarkMeasurement<StereoFeature, Cameras>> left_image_tracks;
    std::vector<LandmarkMeasurement<StereoFeature, Cameras>> right_image_tracks;
};

/// Configuration parameters for the StereoTracker.
class StereoTrackerParams {
 public:
    /** Constructor.
     *
     * @param params_filepath Path to the .YAML file containing the parameters.
     */
    StereoTrackerParams(const std::string &params_filepath);

    /// Default
    ~StereoTrackerParams() = default;
};


template <typename TDetector, typename TDescriptor, typename TMatcher>
class StereoTracker {
 public:
    StereoTracker(TDetector detector, TDescriptor descriptor, TMatcher matcher)
        : detector(detector), descriptor(descriptor), matcher(matcher) {}

    ~StereoTracker() = default;

    /** Get the tracks of all stereo features in the requested image from the
     * sequence.
     *
     * @param stamp The timestamp at which to obtain tracks.
     * @return Tracks corresponding to all detected landmarks in the image, from
     * the start of time to the given image.
     */
    std::vector<StereoFeatureTrack> getTracks(const TimePoint &stamp) const;

    /** Adds a new stereo pair to the tracker (presumably the next in a
     * sequence).
     *
     * @param left_image The image to add from the left camera.
     * @param right_image The image to add from the right camera.
     * @param stamp The time at which the image was captured.
     */
    void addImagePair(const cv::Mat &left_image,
                      const cv::Mat &right_image,
                      const TimePoint &stamp);

    /** Draw tracks on the requested image pair.
     *
     * @param feature_tracks The vector of StereoFeatureTracks present in the
     * image.
     * @param left_image The left image of the stereo pair.
     * @param right_image  The right image of the stereo pair.
     * @return A combined image of the stereo pair with illustrated feature
     * tracks.
     */
    cv::Mat drawTracks(const std::vector<StereoFeatureTrack> &feature_tracks,
                       const cv::Mat &left_image,
                       const cv::Mat &right_image);

 public:
    /// The templated FeatureDetector
    TDetector detector;

    /// The templated DescriptorExtractor
    TDescriptor descriptor;

    /// The templated DescriptorMatcher
    TMatcher matcher;

    /// The size of the LandmarkMeasurementContainer
    size_t lmc_size = 0;

 private:
    /** Generate a new ID for each newly detected feature.
     *
     * @return The assigned ID.
     */
    uint64_t generateFeatureID() const {
        static uint64_t id = 0;
        return id++;
    }

    /** Detect features and compute descriptors for the stereo pair. This is
     * performed for the same instance in time.
     *
     * @param left_image The left image of the stereo pair for detection.
     * @param right_image The right image of the stereo pair for detection.
     * @param keypoints The pair of keypoints
     * @param descriptors
     */
    void detectAndCompute(const cv::Mat &left_image,
                          const cv::Mat &right_image,
                          std::pair<std::vector<cv::KeyPoint>,
                                    std::vector<cv::KeyPoint>> &keypoints,
                          std::pair<cv::Mat, cv::Mat> &descriptors);

    /** Cleans out the LandmarkMeasurementContainer for images outside the
     * requested window_size.
     */
    void maintainContainers();

    /** Registers the latest matched keypoints with IDs. Assigns a new ID if one
     * has not already been provided.
     *
     * @param curr_kp the keypoints detected in the current image.
     * @param matches the matches between the current and previous images.
     * @return the map corresponding current keypoints to IDs.
     */
    std::map<int, LandmarkId> registerKeypoints(
      const std::vector<cv::KeyPoint> &curr_kp,
      const std::vector<cv::DMatch> &matches);

 private:
    /// Camera Intrinsic Matrix
    Eigen::Matrix3d K;

    /// Stereo camera extrinsic matrix;
    Eigen::Affine3d T_left_right;

    /** For online, sliding window tracker operation. Maintains memory by
    *  clearing out values from the measurement container that are outside of
    *  this time window.
    *
    *  If set to zero (default), all measurements are kept for offline use.
    */
    size_t window_size;

    /// Keypoints from the previous timestep.
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> prev_kps;

    /// Descriptors from the previous timestep.
    std::pair<cv::Mat, cv::Mat> prev_descs;

    /// Correspondence map between keypoints and landmark IDs in the prev image.
    std::map<int, LandmarkId> prev_ids;

    /// Times at which each image occurred.
    std::vector<TimePoint> image_stamps;

    /// Landmark measurement container
    LandmarkMeasurementContainer<LandmarkMeasurement<StereoFeature, Cameras>>
      landmarks;
};

/** @} group vision */
}  // namespace wave

#include "impl/stereo_tracker.hpp"

#endif  // WAVE_STEREO_TRACKER_HPP
