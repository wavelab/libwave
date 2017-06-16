// Libwave Headers
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

// Default constructor. Struct may be default or user defined.
BruteForceMatcher::BruteForceMatcher(const BFMatcherParams &config) {
    bool cross_check;

    // Ensure parameters are valid
    this->checkConfiguration(config);

    // If using knn search, cross_check must be false
    if (config.use_knn) {
        cross_check = false;
    } else {
        cross_check = true;
    }

    // Create cv::BFMatcher object with the desired parameters
    this->brute_force_matcher =
      cv::BFMatcher::create(config.norm_type, cross_check);

    // Store configuration parameters within member struct
    this->current_config = config;
}

BruteForceMatcher::BruteForceMatcher(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    // Configuration parameters
    BFMatcherParams config;

    bool cross_check;

    // Add parameters to parser, to be loaded. If path cannot be found, throw an
    // exception.
    parser.addParam("norm_type", &config.norm_type);
    parser.addParam("use_knn", &config.use_knn);
    parser.addParam("ratio_threshold", &config.ratio_threshold);
    parser.addParam("distance_threshold", &config.distance_threshold);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to load BruteForceMatcher configuration! Path not found.");
    }

    // Confirm configuration is valid
    this->checkConfiguration(config);

    // If using knn search, cross_check must be false
    if (config.use_knn) {
        cross_check = false;
    } else {
        cross_check = true;
    }

    // Create cv::BFMatcher object with the desired parameters
    this->brute_force_matcher =
      cv::BFMatcher::create(config.norm_type, cross_check);

    // Store configuration parameters within member struct
    this->current_config = config;
}

void BruteForceMatcher::checkConfiguration(
  const BFMatcherParams &check_config) {
    // Check that the value of norm_type is one of the valid values
    if (check_config.norm_type < cv::NORM_INF ||
        check_config.norm_type > cv::NORM_HAMMING2 ||
        check_config.norm_type == 3) {
        throw std::invalid_argument(
          "Norm type is not one of the acceptable values!");
    }

    // Check the value of the ratio_test heuristic
    if (check_config.ratio_threshold < 0.0 ||
        check_config.ratio_threshold > 1.0) {
        throw std::invalid_argument(
          "ratio_threshold is not an appropriate value!");
    }

    // Check the value of the threshold distance heuristic
    if (check_config.distance_threshold < 0) {
        throw std::invalid_argument("distance_threshold is a negative value!");
    }
}

std::vector<cv::DMatch> BruteForceMatcher::removeOutliers(
  std::vector<cv::DMatch> &matches) const {
    std::vector<cv::DMatch> good_matches;
    float min_distance;

    // Sort matches by distance in increasing order
    std::sort(matches.begin(), matches.end());

    min_distance = matches.begin()->distance;

    // Keep any match that is less than the rejection heuristic times minimum
    // distance
    for (auto &match : matches) {
        if (match.distance <=
            this->current_config.distance_threshold * min_distance) {
            good_matches.push_back(match);
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> BruteForceMatcher::removeOutliers(
  std::vector<std::vector<cv::DMatch>> &matches) const {
    std::vector<cv::DMatch> good_matches;
    float ratio;

    for (auto &match : matches) {
        // Calculate ratio between two best matches. Accept if less than
        // ratio heuristic
        ratio = match[0].distance / match[1].distance;
        if (ratio <= this->current_config.ratio_threshold) {
            good_matches.push_back(match[0]);
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> BruteForceMatcher::matchDescriptors(
  const cv::Mat &descriptors_1, const cv::Mat &descriptors_2) const {
    std::vector<cv::DMatch> good_matches;

    /** Mask variable, currently unused.
     *
     *  The mask variable indicates which descriptors can be matched between the
     *  two sets. As per OpenCV docs "queryDescriptors[i] can be matched with
     *  trainDescriptors[j] only if masks.at<uchar>(i,j) is non-zero.
     *
     *  In the libwave wrapper, queryDescriptors and trainDescriptors are
     *  referred to as descriptors_1 and descriptors_2.
     */
    cv::InputOutputArray mask = cv::noArray();

    if (current_config.use_knn) {
        std::vector<std::vector<cv::DMatch>> matches;
        int k = 2;

        this->brute_force_matcher->knnMatch(
          descriptors_1, descriptors_2, matches, k, mask, false);

        good_matches = this->removeOutliers(matches);

    } else {
        std::vector<cv::DMatch> matches;

        // Determine matches between sets of descriptors
        this->brute_force_matcher->match(
          descriptors_1, descriptors_2, matches, mask);

        good_matches = this->removeOutliers(matches);
    }

    return good_matches;
}

}  // namespace wave
