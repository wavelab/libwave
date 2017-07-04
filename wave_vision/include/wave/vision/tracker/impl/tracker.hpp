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
std::vector<cv::Mat>
Tracker<TDetector, TDescriptor, TMatcher>::drawFeatureTracks(
  const std::vector<std::vector<FeatureTrack>> &feature_tracks,
  const std::vector<cv::Mat> &images) {
    std::vector<cv::Mat> output_images = images;
    std::vector<cv::Mat>::iterator img_it = output_images.begin()++;

    // Define colour for arrows
    cv::Scalar colour(255, 255, 0, 1);

    for (const auto &img_tracks : feature_tracks) {
        cv::Mat out_img = *img_it;

        for (const auto &ft : img_tracks) {
            std::vector<cv::Point2f> points = ft.measurement;
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

template <typename TDetector, typename TDescriptor, typename TMatcher>
std::vector<std::vector<FeatureTrack>>
Tracker<TDetector, TDescriptor, TMatcher>::offlineTracker(
  const std::vector<cv::Mat> &image_sequence) {
    // Variables for detection and matching
    std::vector<cv::KeyPoint> prev_kp;
    std::vector<cv::KeyPoint> curr_kp;
    cv::Mat prev_desc;
    cv::Mat curr_desc;
    std::vector<cv::DMatch> matches;

    std::vector<cv::Mat>::const_iterator image_it;

    // Variables for ID bookkeeping

    // Maps corresponding keypoint indices to IDs
    std::map<int, size_t> prev_ids;
    std::map<int, size_t> curr_ids;
    size_t img_count = 0;

    // Map corresponding IDs with FeatureTrack objects
    std::map<size_t, FeatureTrack> id_map;
    std::map<size_t, FeatureTrack>::iterator id_map_it;
    std::vector<FeatureTrack> curr_track;  // Current feature track
    std::vector<std::vector<FeatureTrack>>
      feature_tracks;  // All feature tracks

    if (!image_sequence.empty()) {
        // Extract keypoints and descriptors from first image, store values
        image_it = image_sequence.begin();
        this->detectAndCompute(*image_it, prev_kp, prev_desc);
        ++image_it;

        // For remaining images
        for (image_it = image_it; image_it != image_sequence.end();
             ++image_it) {
            this->detectAndCompute(*image_it, curr_kp, curr_desc);
            matches = this->matcher.matchDescriptors(
              prev_desc, curr_desc, prev_kp, curr_kp);

            for (const auto &m : matches) {
                size_t id;

                // Check to see if ID has already been assigned to keypoint
                if (prev_ids.find(m.queryIdx) != prev_ids.end()) {
                    // If so, assign that ID to current map.
                    curr_ids[m.trainIdx] = prev_ids.at(m.queryIdx);
                } else {
                    // Else, assign new ID
                    prev_ids[m.queryIdx] = this->generateFeatureID();
                    curr_ids[m.trainIdx] = prev_ids.at(m.queryIdx);
                }

                // Check if ID has associated FeatureTrack.
                id = curr_ids[m.trainIdx];
                if (id_map.find(id) != id_map.end()) {
                    // If track exists, update measurements and last_img count
                    id_map.at(id).measurement.push_back(curr_kp[m.trainIdx].pt);
                    id_map.at(id).last_image = img_count;
                } else {
                    FeatureTrack new_track;

                    new_track.id = id;
                    new_track.measurement.push_back(curr_kp[m.trainIdx].pt);
                    new_track.first_image = img_count;
                    new_track.last_image = new_track.first_image;

                    // Add new track to the id correspondence map
                    id_map[id] = new_track;
                }

                // Add associated track to current tracks in image.
                curr_track.push_back(id_map[id]);
            }

            // Prune id_map of any expired feature tracks
            for (id_map_it = id_map.begin(); id_map_it != id_map.end();
                 ++id_map_it) {
                // If the last image seen is NOT the current image
                if (id_map_it->second.last_image != img_count) {
                    id_map.erase(id_map_it);
                }
            }

            // Add current image tracks to the list of feature tracks, and reset
            feature_tracks.push_back(curr_track);
            curr_track.clear();

            // Set previous ID map to be the current one, and reset
            prev_ids = curr_ids;
            curr_ids.clear();

            // Update previous descriptors
            prev_kp = curr_kp;
            prev_desc = curr_desc;

            ++img_count;
        }
    } else {
        throw std::length_error("No images loaded for image stream!");
    }

    return feature_tracks;
}
}  // namespace wave
