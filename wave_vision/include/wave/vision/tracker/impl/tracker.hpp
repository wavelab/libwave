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
std::map<int, size_t>
Tracker<TDetector, TDescriptor, TMatcher>::registerKeypoints(
  const std::vector<cv::KeyPoint> &curr_kp,
  const std::vector<cv::DMatch> &matches) {
    // Maps current keypoint indices to IDs
    std::map<int, size_t> curr_ids;

    // TODO will need way to specify camera sensor ID.
    int sensor_id = 0;

    for (const auto &m : matches) {
        // Check to see if ID has already been assigned to keypoint
        if (this->prev_ids.find(m.queryIdx) != this->prev_ids.end()) {
            // If so, assign that ID to current map.
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);

            // Extract value of keypoint.
            Vec2 landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            // Create LandmarkMeasurement, and add to
            // LandmarkMeasurementContainer
            LandmarkMeasurement<int> measurement(
              this->img_times.at(this->img_count),
              sensor_id,
              curr_ids.at(m.trainIdx),
              landmark);
            this->landmarks.insert(measurement);
        } else {
            // Else, assign new ID
            this->prev_ids[m.queryIdx] = this->generateFeatureID();
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);

            // Since keypoint was not a match before, need to add previous and
            // current points to measurement container
            Vec2 prev_landmark = convertKeypoint(this->prev_kp.at(m.queryIdx));
            Vec2 curr_landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            std::chrono::steady_clock::time_point prev_time =
              this->img_times.at(this->img_count - 1);
            std::chrono::steady_clock::time_point curr_time =
              this->img_times.at(this->img_count);

            // Create landmark measurements for previous and current landmarks
            LandmarkMeasurement<int> prev_measurement(
              prev_time,
              sensor_id,
              this->prev_ids.at(m.queryIdx),
              prev_landmark);
            LandmarkMeasurement<int> curr_measurement(
              curr_time, sensor_id, curr_ids.at(m.trainIdx), curr_landmark);

            // Add to container
            this->landmarks.insert(prev_measurement);
            this->landmarks.insert(curr_measurement);
        }
    }

    return curr_ids;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<FeatureTrack> Tracker<TDetector, TDescriptor, TMatcher>::getTracks(
  const size_t &img_num) {
    std::vector<FeatureTrack> feature_tracks;
    std::vector<size_t> landmark_ids;

    // TODO add in ability to choose different camera.
    int sensor_id = 0;

    if (img_num > 0) {
        // Find the time for this image
        std::chrono::steady_clock::time_point img_time =
          this->img_times.at(img_num);

        // Extract all of the IDs visible at this time
        landmark_ids =
          this->landmarks.getLandmarkIDsInWindow(img_time, img_time);

        // For each ID, get the track.
        for (const auto &l : landmark_ids) {
            // Looking for track from start of tracking.
            std::chrono::steady_clock::time_point start_time =
              (this->img_times.begin())->second;

            std::vector<LandmarkMeasurement<int>> id_track =
              this->landmarks.getTrackInWindow(
                sensor_id, l, start_time, img_time);

            // Find image corresponding to start time
            std::map<size_t,
                     std::chrono::steady_clock::time_point>::const_iterator
              times;
            std::vector<LandmarkMeasurement<int>>::const_iterator first =
              id_track.begin();

            size_t first_image;
            // Last image feature was seen in must be image requested.
            size_t last_image = img_num;

            for (times = this->img_times.begin();
                 times != this->img_times.end();
                 ++times) {
                if (times->second == first->time_point) {
                    first_image = times->first;
                }
            }

            // Create feature track for ID, and emplace back into vector
            FeatureTrack curr_track(id_track, first_image, last_image);
            feature_tracks.emplace_back(curr_track);
        }
    } else if (img_num == 0) {
        feature_tracks = {};
    } else {
        throw std::invalid_argument("Image index cannot be a negative value!");
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
void Tracker<TDetector, TDescriptor, TMatcher>::addImage(const cv::Mat &image) {
    // Get time for keypoint registration
    std::chrono::steady_clock::time_point current_time = clock.now();

    this->timestampImage(current_time);

    // Check if this is the first image being tracked.
    if (this->img_count == 0) {
        // Detect features within first image. No tracks can be generated yet.
        this->detectAndCompute(image, this->prev_kp, this->prev_desc);
        this->img_count++;
    } else {
        // Variables for feature detection, description, and matching
        std::vector<cv::KeyPoint> curr_kp;
        cv::Mat curr_desc;
        std::vector<cv::DMatch> matches;

        // Variables for bookkeeping
        std::map<int, size_t> curr_ids;

        // Detect, describe, and match keypoints
        this->detectAndCompute(image, curr_kp, curr_desc);
        matches = this->matcher.matchDescriptors(
          this->prev_desc, curr_desc, this->prev_kp, curr_kp);

        // Register keypoints with IDs, and store Landmarks in container
        curr_ids = this->registerKeypoints(curr_kp, matches);

        // Set previous ID map to be the current one, and reset
        this->prev_ids = curr_ids;

        // Update previous keypoints and descriptors
        this->prev_kp = curr_kp;
        this->prev_desc = curr_desc;

        this->img_count++;
    }
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<std::vector<FeatureTrack>>
Tracker<TDetector, TDescriptor, TMatcher>::offlineTracker(
  const std::vector<cv::Mat> &image_sequence) {
    std::vector<cv::Mat>::const_iterator img_it;

    // Current and all feature tracks
    std::vector<FeatureTrack> curr_track;
    std::vector<std::vector<FeatureTrack>> feature_tracks;

    int num_images = 0;

    if (!image_sequence.empty()) {
        for (img_it = image_sequence.begin(); img_it != image_sequence.end();
             ++img_it) {
            // Add image to tracker
            addImage(*img_it);

            // Get tracks from this image (first should return a null track)
            curr_track = this->getTracks(num_images);

            // Add current image tracks to the list of feature tracks
            feature_tracks.push_back(curr_track);
            ++num_images;
        }
    } else {
        throw std::invalid_argument("No images loaded for image stream!");
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
cv::Mat Tracker<TDetector, TDescriptor, TMatcher>::drawTracks(
  const size_t &img_num, const cv::Mat &image) {
    cv::Mat out_img = image;

    // Get the tracks for this image
    std::vector<FeatureTrack> feature_tracks = this->getTracks(img_num);

    // Define colour for arrows
    cv::Scalar colour(0, 255, 255);  // yellow

    size_t max_size = 0;

    // Draw all feature tracks on out_img
    for (const auto &ft : feature_tracks) {
        if (ft.size() > max_size) {
            max_size = ft.size();
        }

        for (size_t i = 1; i < ft.size(); i++) {
            // Convert landmark values to cv::Point2f
            cv::Point2f prev = convertKeypoint(ft.measurements[i - 1].value);
            cv::Point2f curr = convertKeypoint(ft.measurements[i].value);

            // Draw arrowed line until end of feature track is reached
            cv::arrowedLine(out_img, prev, curr, colour);
        }
    }

    return out_img;
}
}  // namespace wave
