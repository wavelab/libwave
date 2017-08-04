#include "wave/vision/tracker/tracker.hpp"

namespace wave {

// Private Functions
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

    // Every 10 frames, clear the measurement container
    std::cout << "Landmark MC size: " << this->landmarks.size() << std::endl;
    std::cout << "img_times size: " << this->img_times.size() << std::endl;
    auto img_count = this->img_times.size();
    if (img_count % 10 == 0) {
        auto lndmrk_begin = this->landmarks.begin();
        auto lndmrk_last = this->landmarks.end();
        --lndmrk_last;

        this->landmarks.erase(lndmrk_begin, lndmrk_last);
        std::cout << "Landmarks cleared, size is: " << this->landmarks.size()
                  << std::endl;
    }

    for (const auto &m : matches) {
        // Check to see if ID has already been assigned to keypoint
        if (this->prev_ids.count(m.queryIdx)) {
            // If so, assign that ID to current map.
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);

            // Extract value of keypoint.
            Vec2 landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            auto img_count = this->img_times.size() - 1;

            // Emplace LandmarkMeasurement into LandmarkMeasurementContainer
            this->landmarks.emplace(this->img_times.at(img_count),
                                    sensor_id,
                                    curr_ids.at(m.trainIdx),
                                    img_count,
                                    landmark);
        } else {
            // Else, assign new ID
            this->prev_ids[m.queryIdx] = this->generateFeatureID();
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);

            // Since keypoint was not a match before, need to add previous and
            // current points to measurement container
            Vec2 prev_landmark = convertKeypoint(this->prev_kp.at(m.queryIdx));
            Vec2 curr_landmark = convertKeypoint(curr_kp.at(m.trainIdx));

            // Find previous and current times from lookup table
            // Subtract one, since images are zero indexed.
            auto curr_img = this->img_times.size() - 1;
            auto prev_img = curr_img - 1;

            const auto &prev_time = this->img_times.at(prev_img);
            const auto &curr_time = this->img_times.at(curr_img);

            // Add previous and current landmarks to container
            this->landmarks.emplace(prev_time,
                                    sensor_id,
                                    this->prev_ids.at(m.queryIdx),
                                    prev_img,
                                    prev_landmark);

            this->landmarks.emplace(curr_time,
                                    sensor_id,
                                    curr_ids.at(m.trainIdx),
                                    curr_img,
                                    curr_landmark);
        }
    }

    return curr_ids;
}

// Public Functions
template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<FeatureTrack> Tracker<TDetector, TDescriptor, TMatcher>::getTracks(
  const size_t &img_num) const {
    std::vector<FeatureTrack> feature_tracks;

    // TODO add in ability to choose different camera.
    int sensor_id = 0;

    // Determine how many images have been added
    auto img_count = this->img_times.size() - 1;

    if (img_num > img_count) {
        throw std::out_of_range("Image requested is in the future!");
    } else if (img_num > 0) {
        // Find the time for this image
        std::chrono::steady_clock::time_point img_time =
          this->img_times.at(img_num);

        // Extract all of the IDs visible at this time
        auto landmark_ids =
          this->landmarks.getLandmarkIDsInWindow(img_time, img_time);

        // For each ID, get the track.
        for (const auto &l : landmark_ids) {
            // Looking for track from first image.
            std::chrono::steady_clock::time_point start_time =
              (this->img_times.begin())->second;

            FeatureTrack tracks = this->landmarks.getTrackInWindow(
              sensor_id, l, start_time, img_time);

            // Emplace new feature track back into vector
            feature_tracks.emplace_back(tracks);
        }
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
void Tracker<TDetector, TDescriptor, TMatcher>::addImage(
  const cv::Mat &image,
  const std::chrono::steady_clock::time_point &current_time) {
    // Register the time this image
    this->timestampImage(current_time);

    // Check if this is the first image being tracked.
    if (this->img_times.size() == 1) {
        // Detect features within first image. No tracks can be generated yet.
        this->detectAndCompute(image, this->prev_kp, this->prev_desc);
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

    int num_images = 0;

    if (!image_sequence.empty()) {
        for (auto img_it = image_sequence.begin();
             img_it != image_sequence.end();
             ++img_it) {
            // Add image to tracker
            this->addImage(*img_it, clock.now());

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
            cv::arrowedLine(out_img, prev, curr, colour);
        }
    }

    return out_img;
}
}  // namespace wave
