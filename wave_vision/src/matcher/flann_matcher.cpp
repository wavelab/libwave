#include "wave/vision/matcher/flann_matcher.hpp"

namespace wave {

// Filesystem based constructor for FLANNMatcherParams
FLANNMatcherParams::FLANNMatcherParams(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int flann_method;
    bool use_knn;
    double ratio_threshold;
    int distance_threshold;
    bool auto_remove_outliers;
    int fm_method;

    // Add parameters to parser, to be loaded. If path cannot be found, throw
    // an exception.
    parser.addParam("flann_method", &flann_method);
    parser.addParam("use_knn", &use_knn);
    parser.addParam("ratio_threshold", &ratio_threshold);
    parser.addParam("distance_threshold", &distance_threshold);
    parser.addParam("auto_remove_outliers", &auto_remove_outliers);
    parser.addParam("fm_method", &fm_method);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to Load FLANNMatcherParams Configuration");
    }

    this->flann_method = flann_method;
    this->use_knn = use_knn;
    this->ratio_threshold = ratio_threshold;
    this->distance_threshold = distance_threshold;
    this->auto_remove_outliers = auto_remove_outliers;
    this->fm_method = fm_method;
}

FLANNMatcher::FLANNMatcher(const FLANNMatcherParams &config) {
    // Check flann_method and create the appropriate parameters struct.
    this->checkConfiguration(config);

    // Depending on the FLANN method, different parameters are required.
    if (config.flann_method == FLANN::KDTree) {
        // Create FLANN matcher with default KDTree and Search params.
        cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(),
                                      new cv::flann::SearchParams());
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::KMeans) {
        // Create FLANN matcher with default KMeans and Search params
        cv::FlannBasedMatcher matcher(new cv::flann::KMeansIndexParams(),
                                      new cv::flann::SearchParams());
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::Composite) {
        // Create FLANN matcher with default Composite and Search params
        cv::FlannBasedMatcher matcher(new cv::flann::CompositeIndexParams(),
                                      new cv::flann::SearchParams());
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::LSH) {
        // Create LSH params with default values. These are values recommended
        // by Kaehler and Bradski - the LSH struct in OpenCV does not have
        // default values unlike the others.
        unsigned int num_tables = 20;  // Typically between 10-30
        unsigned int key_size = 15;    // Typically between 10-20
        unsigned int multi_probe_level = 2;

        // Create FLANN matcher with default LSH and Search params
        cv::FlannBasedMatcher matcher(
          new cv::flann::LshIndexParams(
            num_tables, key_size, multi_probe_level),
          new cv::flann::SearchParams());
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    }

    this->current_config = config;
}

void FLANNMatcher::checkConfiguration(const FLANNMatcherParams &check_config) {
    // Check that the value of flann_method is one of the valid values.
    if (check_config.flann_method < FLANN::KDTree ||
        check_config.flann_method > FLANN::LSH) {
        throw std::invalid_argument("Flann method selected does not exist!");
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

std::vector<cv::DMatch> FLANNMatcher::filterMatches(
  std::vector<cv::DMatch> &matches) const {
    std::vector<cv::DMatch> filtered_matches;

    // Determine closest match
    auto closest_match = std::min_element(matches.begin(), matches.end());
    auto min_distance = closest_match->distance;

    // Keep any match that is less than the rejection heuristic times minimum
    // distance
    for (auto &match : matches) {
        if (match.distance <=
            this->current_config.distance_threshold * min_distance) {
            filtered_matches.push_back(match);
        }
    }

    return filtered_matches;
}

std::vector<cv::DMatch> FLANNMatcher::filterMatches(
  std::vector<std::vector<cv::DMatch>> &matches) const {
    std::vector<cv::DMatch> filtered_matches;

    for (auto &match : matches) {
        // Calculate ratio between two best matches. Accept if less than
        // ratio heuristic
        float ratio = match[0].distance / match[1].distance;
        if (ratio <= this->current_config.ratio_threshold) {
            filtered_matches.push_back(match[0]);
        }
    }

    return filtered_matches;
}

std::vector<cv::DMatch> FLANNMatcher::removeOutliers(
  const std::vector<cv::DMatch> &matches,
  const std::vector<cv::KeyPoint> &keypoints_1,
  const std::vector<cv::KeyPoint> &keypoints_2) const {
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> fp1, fp2;

    // Take all good keypoints from matches, convert to cv::Point2f
    for (auto &match : matches) {
        fp1.push_back(keypoints_1.at((size_t) match.queryIdx).pt);
        fp2.push_back(keypoints_2.at((size_t) match.trainIdx).pt);
    }

    // Find fundamental matrix
    std::vector<uchar> mask;
    cv::Mat fundamental_matrix;

    // Maximum distance from a point to an epipolar line in pixels. Any points
    // further are considered outliers. Only used for RANSAC.
    double fm_param_1 = 3.0;

    // Desired confidence interval of the estimated fundamental matrix. Only
    // used for RANSAC or LMedS methods.
    double fm_param_2 = 0.99;

    fundamental_matrix = cv::findFundamentalMat(
      fp1, fp2, this->current_config.fm_method, fm_param_1, fm_param_2, mask);

    // Only retain the inliers matches
    for (size_t i = 0; i < mask.size(); i++) {
        if (mask.at(i) != 0) {
            good_matches.push_back(matches.at(i));
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> FLANNMatcher::matchDescriptors(
  cv::Mat &descriptors_1,
  cv::Mat &descriptors_2,
  const std::vector<cv::KeyPoint> &keypoints_1,
  const std::vector<cv::KeyPoint> &keypoints_2,
  cv::InputArray mask) {
    std::vector<cv::DMatch> filtered_matches;

    // The FLANN matcher (except for the LSH method) requires the descriptors
    // to be of type CV_32F (float, from 0-1.0). Some descriptors
    // (ex. ORB, BRISK) provide descriptors in the form of CV_8U (unsigned int).
    // To use the other methods, the descriptor must be converted before
    // matching.
    if (this->current_config.flann_method != FLANN::LSH &&
        descriptors_1.type() != CV_32F) {
        descriptors_1.convertTo(descriptors_1, CV_32F);
    }

    if (this->current_config.flann_method != FLANN::LSH &&
        descriptors_2.type() != CV_32F) {
        descriptors_2.convertTo(descriptors_2, CV_32F);
    }

    if (this->current_config.use_knn) {
        std::vector<std::vector<cv::DMatch>> raw_matches;

        // Number of neighbours for the k-nearest neighbour search. Only used
        // for the ratio test, therefore only want 2.
        int k = 2;

        this->flann_matcher->knnMatch(
          descriptors_1, descriptors_2, raw_matches, k, mask, false);
        this->num_raw_matches = raw_matches.size();

        filtered_matches = this->filterMatches(raw_matches);
        this->num_filtered_matches = filtered_matches.size();
    } else {
        std::vector<cv::DMatch> raw_matches;

        // Determine matches between sets of descriptors
        this->flann_matcher->match(
          descriptors_1, descriptors_2, raw_matches, mask);
        this->num_raw_matches = raw_matches.size();

        filtered_matches = this->filterMatches(raw_matches);
        this->num_filtered_matches = filtered_matches.size();
    }

    if (this->current_config.auto_remove_outliers) {
        std::vector<cv::DMatch> good_matches =
          this->removeOutliers(filtered_matches, keypoints_1, keypoints_2);
        this->num_good_matches = good_matches.size();

        return good_matches;
    }

    return filtered_matches;
}
}  // namespace wave
