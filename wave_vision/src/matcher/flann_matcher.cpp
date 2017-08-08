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

    // Create search params with default values. This is used for all
    // matcher implementations.
    cv::flann::SearchParams search_params;

    // Depending on the FLANN method, different parameters are required.
    if (config.flann_method == FLANN::KDTree) {
        // Create KDTree index params and search params with default values.
        cv::flann::KDTreeIndexParams kdtree_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(
          cv::makePtr<cv::flann::KDTreeIndexParams>(kdtree_params),
          cv::makePtr<cv::flann::SearchParams>(search_params));
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::KMeans) {
        // Create KMeans index params with default values.
        cv::flann::KMeansIndexParams kmeans_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(
          cv::makePtr<cv::flann::KMeansIndexParams>(kmeans_params),
          cv::makePtr<cv::flann::SearchParams>(search_params));
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::Composite) {
        // Create composite index params with default values.
        cv::flann::CompositeIndexParams composite_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(
          cv::makePtr<cv::flann::CompositeIndexParams>(composite_params),
          cv::makePtr<cv::flann::SearchParams>(search_params));
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::LSH) {
        // Create LSH params with default values. These are values recommended
        // by Kaehler and Bradski - the LSH struct in OpenCV does not have
        // default values unlike the others.
        unsigned int num_tables = 20;  // Typically between 10-30
        unsigned int key_size = 15;    // Typically between 10-20
        unsigned int multi_probe_level = 2;
        cv::flann::LshIndexParams lsh_params(
          num_tables, key_size, multi_probe_level);

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(
          cv::makePtr<cv::flann::LshIndexParams>(lsh_params),
          cv::makePtr<cv::flann::SearchParams>(search_params));
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    } else if (config.flann_method == FLANN::Autotuned) {
        // Create autotuned index params with default values
        cv::flann::AutotunedIndexParams autotuned_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(
          cv::makePtr<cv::flann::AutotunedIndexParams>(autotuned_params),
          cv::makePtr<cv::flann::SearchParams>(search_params));
        this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
    }

    this->current_config = config;
}

void FLANNMatcher::checkConfiguration(const FLANNMatcherParams &check_config) {
    // Check that the value of flann_method is one of the valid values.
    if (check_config.flann_method < FLANN::KDTree ||
        check_config.flann_method > FLANN::Autotuned) {
        throw std::invalid_argument("Flann method selected does not exist!");
    }

    // If use_knn is true, check ratio_threshold value. Otherwise check
    // distance_threshold
    if (check_config.use_knn) {
        // Check the value of the ratio_test heuristic
        if (check_config.ratio_threshold < 0.0 ||
            check_config.ratio_threshold > 1.0) {
            throw std::invalid_argument(
              "ratio_threshold is not an appropriate value!");
        }
    } else {
        // Check the value of the threshold distance heuristic
        if (check_config.distance_threshold < 0) {
            throw std::invalid_argument(
              "distance_threshold is a negative value!");
        }
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
}