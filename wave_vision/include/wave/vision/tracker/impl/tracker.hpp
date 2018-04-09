#include "wave/vision/tracker/tracker.hpp"

namespace wave {

template <typename TDetector, typename TDescriptor, typename TMatcher>
void Tracker<TDetector, TDescriptor, TMatcher>::detectAndCompute(
  const cv::Mat &image,
  std::vector<cv::KeyPoint> &keypoints,
  cv::Mat &descriptor) {
    keypoints = this->detector.detectFeatures(image);
    descriptor = this->descriptor.extractDescriptors(image, keypoints);
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
void Tracker<TDetector, TDescriptor, TMatcher>::maintainContainers() {
    auto num_images_to_clear =
      static_cast<int>(this->image_stamps.size() - this->window_size);

    if (num_images_to_clear > 0) {
        for (int i = 0; i < num_images_to_clear; ++i) {
            auto stamp = this->image_stamps.at(0);

            // Get all IDs at this time
            auto landmarks =
              this->landmarks.getLandmarkIDsInWindow(stamp, stamp);

            // Delete all landmarks in the container at this time.
            for (const auto &l : landmarks) {
                this->landmarks.erase(stamp, this->sensor_id, l);
            }

            // Delete first element of image stamps
            this->image_stamps.erase(this->image_stamps.begin());
        }
    }
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::map<int, LandmarkId>
Tracker<TDetector, TDescriptor, TMatcher>::registerKeypoints(
  const std::vector<cv::KeyPoint> &curr_kp,
  const std::vector<cv::DMatch> &matches) {
    // Maps current keypoint indices to IDs
    std::map<int, LandmarkId> curr_ids;

    for (const auto &m : matches) {
        // Check to see if ID has already been assigned to keypoint
        if (this->prev_ids.count(m.queryIdx)) {
            // If so, assign that ID to current map.
            auto id = this->prev_ids.at(m.queryIdx);
            curr_ids[m.trainIdx] = id;

            // Extract value of keypoint.
            Vec2 landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            // Emplace LandmarkMeasurement into LandmarkMeasurementContainer
            this->landmarks.emplace(this->image_stamps.back(),
                                    this->sensor_id,
                                    curr_ids.at(m.trainIdx),
                                    landmark);
        } else {
            // Else, assign new ID
            auto id = this->generateFeatureID();
            this->prev_ids[m.queryIdx] = id;
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);

            // Since keypoint was not a match before, need to add previous and
            // current points to measurement container
            Vec2 prev_landmark = convertKeypoint(this->prev_kp.at(m.queryIdx));
            Vec2 curr_landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            // Find previous and current times from lookup table
            // Subtract one, since images are zero indexed.
            auto curr_img = this->image_stamps.size() - 1;
            auto prev_img = curr_img - 1;

            auto prev_time = this->image_stamps.at(prev_img);
            auto curr_time = this->image_stamps.at(curr_img);

            // Add previous and current landmarks to container
            this->landmarks.emplace(prev_time,
                                    this->sensor_id,
                                    this->prev_ids.at(m.queryIdx),
                                    prev_landmark);

            this->landmarks.emplace(curr_time,
                                    this->sensor_id,
                                    curr_ids.at(m.trainIdx),
                                    curr_landmark);
        }
    }

    // If in online mode, need to maintain container sizes according to window
    if (this->window_size > 0) {
        this->maintainContainers();
    }

    this->lmc_size = this->landmarks.size();

    return curr_ids;
}

// Public Functions
template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<FeatureTrack> Tracker<TDetector, TDescriptor, TMatcher>::getTracks(
  const TimePoint &stamp) const {
    std::vector<FeatureTrack> feature_tracks;

    // See if we have an image for the particular stamp
    if (std::find(this->image_stamps.begin(),
                  this->image_stamps.end(),
                  stamp) == this->image_stamps.end()) {
        if (stamp < *this->image_stamps.begin()) {
            // for non-zero window_size, the measurement container is
            // periodically cleaned out. Can only access images in container.
            throw std::out_of_range(
              "Image requested occurred too early, and is outside of "
              "maintained window!");
        } else if (stamp > this->image_stamps.back()) {
            throw std::out_of_range("Image requested is in the future!");
        }
    } else {
        // Extract all of the IDs visible at this time
        auto landmark_ids =
          this->landmarks.getLandmarkIDsInWindow(stamp, stamp);

        // For each ID, get the track.
        for (const auto &l : landmark_ids) {
            // Looking for track from first image.
            TimePoint start_time = *this->image_stamps.begin();

            FeatureTrack tracks = this->landmarks.getTrackInWindow(
              this->sensor_id, l, start_time, stamp);

            // Emplace new feature track back into vector
            feature_tracks.emplace_back(tracks);
        }
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
void Tracker<TDetector, TDescriptor, TMatcher>::addImage(
  const cv::Mat &image, const TimePoint &stamp) {
    // Register the time of this image
    this->image_stamps.emplace_back(stamp);

    // Check if this is the first image being tracked.
    if (this->image_stamps.size() == 1) {
        // Detect features within first image. No tracks can be generated yet.
        this->detectAndCompute(image, this->prev_kp, this->prev_desc);
    } else {
        // Variables for feature detection, description, and matching
        std::vector<cv::KeyPoint> curr_kp;
        cv::Mat curr_desc;

        // Detect, describe, and match keypoints
        this->detectAndCompute(image, curr_kp, curr_desc);
        std::vector<cv::DMatch> matches = this->matcher.matchDescriptors(
          this->prev_desc, curr_desc, this->prev_kp, curr_kp);

        // Register keypoints with IDs, and store Landmarks in container
        std::map<int, LandmarkId> curr_ids =
          this->registerKeypoints(curr_kp, matches);

        // Set previous ID map to be the current one
        this->prev_ids.swap(curr_ids);

        // Update previous keypoints and descriptors
        this->prev_kp = curr_kp;
        this->prev_desc = curr_desc;
    }
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<std::vector<FeatureTrack>>
Tracker<TDetector, TDescriptor, TMatcher>::offlineTracker(
  const std::vector<cv::Mat> &image_sequence) {
    // FeatureTracks from current image, and all FeatureTracks
    std::vector<FeatureTrack> curr_track;
    std::vector<std::vector<FeatureTrack>> feature_tracks;

    std::chrono::steady_clock clock;

    if (!image_sequence.empty()) {
        for (const auto &img : image_sequence) {
            auto stamp = clock.now();

            // Add image to tracker
            this->addImage(img, stamp);

            // Get tracks from this image (first should return a null track)
            curr_track = this->getTracks(stamp);

            // Add current image tracks to the list of feature tracks
            feature_tracks.push_back(curr_track);
        }
    } else {
        throw std::invalid_argument("No images loaded for image stream!");
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
cv::Mat Tracker<TDetector, TDescriptor, TMatcher>::drawTracks(
  const std::vector<FeatureTrack> &feature_tracks, const cv::Mat &image) const {
    cv::Mat out_img = image;

    // Define colour for arrows
    cv::Scalar colour(0, 255, 255);  // yellow

    // Draw all feature tracks on out_img
    for (const auto &ft : feature_tracks) {
        for (size_t i = 1; i < ft.size(); i++) {
            // Convert landmark values to cv::Point2f
            cv::Point2f prev = convertKeypoint(ft[i - 1].value);
            cv::Point2f curr = convertKeypoint(ft[i].value);

            // Draw arrowed line until end of feature track is reached
            cv::arrowedLine(out_img, prev, curr, colour, 2);
        }
    }

    return out_img;
}
}  // namespace wave
