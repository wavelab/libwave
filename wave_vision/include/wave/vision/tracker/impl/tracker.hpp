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
void Tracker<TDetector, TDescriptor, TMatcher>::removeExpiredIDs() {
    std::map<size_t, FeatureTrack>::iterator id_map_it = this->id_map.begin();

    for (id_map_it = this->id_map.begin(); id_map_it != this->id_map.end();) {
        // If the feature was NOT observed in the current image
        if (id_map_it->second.last_image != this->img_count) {
            // Erase value in map
            id_map_it = this->id_map.erase(id_map_it);
        } else {
            // Move to the next value
            id_map_it++;
        }
    }
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::map<int, size_t>
Tracker<TDetector, TDescriptor, TMatcher>::registerKeypoints(
  const std::vector<cv::DMatch> &matches) {
    // Maps current keypoint indices to IDs
    std::map<int, size_t> curr_ids;

    for (const auto &m : matches) {
        // Check to see if ID has already been assigned to keypoint
        if (this->prev_ids.find(m.queryIdx) != this->prev_ids.end()) {
            // If so, assign that ID to current map.
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);
        } else {
            // Else, assign new ID
            this->prev_ids[m.queryIdx] = this->generateFeatureID();
            curr_ids[m.trainIdx] = this->prev_ids.at(m.queryIdx);
        }
    }

    return curr_ids;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<FeatureTrack>
Tracker<TDetector, TDescriptor, TMatcher>::generateFeatureTracks(
  const std::vector<cv::KeyPoint> curr_kp,
  const std::vector<cv::DMatch> matches) {
    // Check if ID has associated FeatureTrack.
    size_t id;
    std::vector<FeatureTrack> curr_track;

    for (const auto &m : matches) {
        id = this->prev_ids.at(m.queryIdx);

        if (this->id_map.find(id) != this->id_map.end()) {
            // If track exists, update measurements and last_img count
            this->id_map.at(id).measurement.push_back(
              curr_kp.at(m.trainIdx).pt);
            this->id_map.at(id).last_image = this->img_count;
        } else {
            FeatureTrack new_track;

            new_track.id = id;
            new_track.measurement.push_back(this->prev_kp.at(m.queryIdx).pt);
            new_track.measurement.push_back(curr_kp.at(m.trainIdx).pt);
            new_track.first_image = this->img_count - 1;
            new_track.last_image = this->img_count;

            // Add new track to the id correspondence map
            this->id_map[id] = new_track;
        }

        // Add associated track to current tracks in image.
        curr_track.push_back(this->id_map.at(id));
    }

    return curr_track;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<FeatureTrack> Tracker<TDetector, TDescriptor, TMatcher>::trackImage(
  const cv::Mat &next_image) {
    std::vector<FeatureTrack> curr_track;

    // Check if this is the first image being tracked.
    if (this->img_count == 0) {
        // Detect features within first image. No tracks can be generated yet.
        this->detectAndCompute(next_image, this->prev_kp, this->prev_desc);
        this->img_count++;
    } else {
        // Variables for feature detection, description, and matching
        std::vector<cv::KeyPoint> curr_kp;
        cv::Mat curr_desc;
        std::vector<cv::DMatch> matches;

        // Variables for bookkeeping
        std::map<int, size_t> curr_ids;

        // Detect, describe, and match keypoints
        this->detectAndCompute(next_image, curr_kp, curr_desc);
        matches = this->matcher.matchDescriptors(
          this->prev_desc, curr_desc, this->prev_kp, curr_kp);

        // Register keypoints with IDs
        curr_ids = this->registerKeypoints(matches);

        // Obtain the new feature tracks
        curr_track = this->generateFeatureTracks(curr_kp, matches);

        // Remove any expired feature tracks
        this->removeExpiredIDs();

        // Set previous ID map to be the current one, and reset
        this->prev_ids = curr_ids;

        // Update previous keypoints and descriptors
        this->prev_kp = curr_kp;
        this->prev_desc = curr_desc;

        this->img_count++;
    }

    return curr_track;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<std::vector<FeatureTrack>>
Tracker<TDetector, TDescriptor, TMatcher>::offlineTracker(
  const std::vector<cv::Mat> &image_sequence) {
    std::vector<cv::Mat>::const_iterator img_it;

    // Current and all feature tracks
    std::vector<FeatureTrack> curr_track;
    std::vector<std::vector<FeatureTrack>> feature_tracks;

    if (!image_sequence.empty()) {
        for (img_it = image_sequence.begin(); img_it != image_sequence.end();
             ++img_it) {
            curr_track = trackImage(*img_it);

            // Add current image tracks to the list of feature tracks
            feature_tracks.push_back(curr_track);
        }
    } else {
        throw std::length_error("No images loaded for image stream!");
    }

    return feature_tracks;
}

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<cv::Mat>
Tracker<TDetector, TDescriptor, TMatcher>::drawFeatureTracks(
  const std::vector<std::vector<FeatureTrack>> &feature_tracks,
  const std::vector<cv::Mat> &images) {
    std::vector<cv::Mat> output_images = images;
    cv::Mat out_img;
    std::vector<cv::Mat>::iterator img_it = output_images.begin();

    // Define iterators for feature tracks
    std::vector<std::vector<FeatureTrack>>::const_iterator curr_track;

    // Define colour for arrows
    cv::Scalar colour(0, 255, 255);  // yellow

    for (curr_track = feature_tracks.begin();
         curr_track != feature_tracks.end();
         ++curr_track) {
        out_img = *img_it;

        // Draw arrows from previous track
        for (const auto &ct : *curr_track) {
            std::vector<cv::Point2f> points = ct.measurement;

            // Draw arrowed lines for each track
            for (size_t i = 1; i < points.size(); ++i) {
                cv::Point2f curr = points[i];
                cv::Point2f prev = points[i - 1];
                cv::arrowedLine(out_img, prev, curr, colour);
            }
        }

        ++img_it;
    }


    return output_images;
}
}  // namespace wave
