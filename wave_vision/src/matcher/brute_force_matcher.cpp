#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

// Filesystem based constructor for BFMatcherParams
BFMatcherParams::BFMatcherParams(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int norm_type;
    bool use_knn;
    double ratio_threshold;
    int distance_threshold;
    bool remove_outliers;
    int fm_method;

    // Add parameters to parser, to be loaded. If path cannot be found, throw
    // an exception.
    parser.addParam("norm_type", &norm_type);
    parser.addParam("use_knn", &use_knn);
    parser.addParam("ratio_threshold", &ratio_threshold);
    parser.addParam("distance_threshold", &distance_threshold);
    parser.addParam("remove_outliers", &remove_outliers);
    parser.addParam("fm_method", &fm_method);

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::invalid_argument(
          "Failed to Load BFMatcherParams Configuration");
    }

    this->norm_type = norm_type;
    this->use_knn = use_knn;
    this->ratio_threshold = ratio_threshold;
    this->distance_threshold = distance_threshold;
    this->remove_outliers = remove_outliers;
    this->fm_method = fm_method;
}

// Default constructor. Struct may be default or user defined.
BruteForceMatcher::BruteForceMatcher(const BFMatcherParams &config,
                                     const FMParams &fm_params) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Cross_check must be the opposite of use_knn
    bool cross_check = !config.use_knn;

    // Create cv::BFMatcher object with the desired parameters
    this->brute_force_matcher =
      cv::BFMatcher::create(config.norm_type, cross_check);

    // Store configuration parameters within member struct
    this->config = config;
    this->fm_params = fm_params;
}

void BruteForceMatcher::checkConfiguration(
  const BFMatcherParams &check_config) const {
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

    // Only acceptable values are 1, 2, 4, and 8
    if (check_config.fm_method != cv::FM_7POINT &&
        check_config.fm_method != cv::FM_8POINT &&
        check_config.fm_method != cv::FM_LMEDS &&
        check_config.fm_method != cv::FM_RANSAC) {
        throw std::invalid_argument("fm_method is not an acceptable value!");
    }
}

std::vector<cv::DMatch> BruteForceMatcher::filterMatches(
  const std::vector<cv::DMatch> &matches) const {
    std::vector<cv::DMatch> filtered_matches;

    // Determine closest match
    auto closest_match = std::min_element(matches.begin(), matches.end());
    auto min_distance = closest_match->distance;

    // Keep any match that is less than the rejection heuristic times minimum
    // distance
    for (auto &match : matches) {
        if (match.distance <= this->config.distance_threshold * min_distance) {
            filtered_matches.push_back(match);
        }
    }

    return filtered_matches;
}

std::vector<cv::DMatch> BruteForceMatcher::filterMatches(
  const std::vector<std::vector<cv::DMatch>> &matches) const {
    std::vector<cv::DMatch> filtered_matches;

    for (auto &match : matches) {
        // Calculate ratio between two best matches. Accept if less than
        // ratio heuristic
        float ratio = match[0].distance / match[1].distance;
        if (ratio <= this->config.ratio_threshold) {
            filtered_matches.push_back(match[0]);
        }
    }

    return filtered_matches;
}

std::vector<cv::DMatch> BruteForceMatcher::removeOutliers(
  const std::vector<cv::DMatch> &matches,
  const std::vector<cv::KeyPoint> &keypoints_1,
  const std::vector<cv::KeyPoint> &keypoints_2) const {
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> pixels1, pixels2;

    // Take all good keypoints from matches, convert to cv::Point2f
    for (auto &match : matches) {
        pixels1.push_back(
          keypoints_1.at(static_cast<size_t>(match.queryIdx)).pt);
        pixels2.push_back(
          keypoints_2.at(static_cast<size_t>(match.trainIdx)).pt);
    }

    // Find fundamental matrix
    std::vector<uchar> mask;
    auto fundamental_matrix = cv::findFundamentalMat(pixels1,
                                                     pixels2,
                                                     this->config.fm_method,
                                                     this->fm_params.max_dist,
                                                     this->fm_params.confidence,
                                                     mask);

    // Only retain the inlier matches
    for (size_t i = 0; i < mask.size(); i++) {
        if (mask.at(i) != 0) {
            good_matches.push_back(matches.at(i));
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> BruteForceMatcher::matchDescriptors(
  cv::Mat &descriptors_1,
  cv::Mat &descriptors_2,
  const std::vector<cv::KeyPoint> &keypoints_1,
  const std::vector<cv::KeyPoint> &keypoints_2,
  cv::InputArray mask) {
    std::vector<cv::DMatch> filtered_matches;

    if (this->config.use_knn) {
        std::vector<std::vector<cv::DMatch>> raw_matches;

        // Number of neighbours for the k-nearest neighbour search. Only used
        // for the ratio test, therefore only want 2.
        int k = 2;

        // Initial matching
        this->brute_force_matcher->knnMatch(
          descriptors_1, descriptors_2, raw_matches, k, mask, false);
        this->num_raw_matches = raw_matches.size();

        // Filter matches
        filtered_matches = this->filterMatches(raw_matches);
        this->num_filtered_matches = filtered_matches.size();
    } else {
        std::vector<cv::DMatch> raw_matches;

        // Initial matching
        this->brute_force_matcher->match(
          descriptors_1, descriptors_2, raw_matches, mask);
        this->num_raw_matches = raw_matches.size();

        // Filter matches
        filtered_matches = this->filterMatches(raw_matches);
        this->num_filtered_matches = filtered_matches.size();
    }

    // If the user wants outliers to be removed (via RANSAC or similar)
    if (this->config.remove_outliers) {
        // Remove outliers.
        std::vector<cv::DMatch> good_matches =
          this->removeOutliers(filtered_matches, keypoints_1, keypoints_2);
        this->num_good_matches = good_matches.size();

        return good_matches;
    }

    return filtered_matches;
}
}  // namespace wave
