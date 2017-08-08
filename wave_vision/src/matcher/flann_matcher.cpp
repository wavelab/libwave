#include "wave/vision/matcher/flann_matcher.hpp"

namespace wave {

FLANNMatcher::FLANNMatcher(int flann_method) {
    // Check flann_method and create the appropriate parameters struct.
    this->checkConfiguration(flann_method);

    // Create search params with default values. This is used for all
    // matcher implementations.
    cv::flann::SearchParams search_params;

    // Depending on the FLANN method, different parameters are required.
    if (flann_method == FLANN::KDTree) {
        // Create KDTree index params and search params with default values.
        cv::flann::KDTreeIndexParams kdtree_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(cv::makePtr(kdtree_params),
                                      cv::makePtr(search_params));
        this->flann_matcher = cv::makePtr(matcher);
    } else if (flann_method == FLANN::KMeans) {
        // Create KMeans index params with default values.
        cv::flann::KMeansIndexParams kmeans_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(cv::makePtr(kmeans_params),
                                      cv::makePtr(search_params));
        this->flann_matcher = cv::makePtr(matcher);

    } else if (flann_method == FLANN::Composite) {
        // Create composite index params with default values.
        cv::flann::CompositeIndexParams composite_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(cv::makePtr(composite_params),
                                      cv::makePtr(search_params));
        this->flann_matcher = cv::makePtr(matcher);
    } else if (flann_method == FLANN::LSH) {
        // Create LSH params with default values. These are values recommended
        // by Kaehler and Bradski - the LSH struct in OpenCV does not have
        // default values unlike the others.
        unsigned int num_tables = 20;  // Typically between 10-30
        unsigned int key_size = 15;    // Typically between 10-20
        unsigned int multi_probe_level = 2;
        cv::flann::LshIndexParams lsh_params(
          num_tables, key_size, multi_probe_level);

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(cv::makePtr(lsh_params),
                                      cv::makePtr(search_params));
        this->flann_matcher = cv::makePtr(matcher);
    } else if (flann_method == FLANN::Autotuned) {
        // Create autotuned index params with default values
        cv::flann::AutotunedIndexParams autotuned_params;

        // Create FLANN matcher
        cv::FlannBasedMatcher matcher(cv::makePtr(autotuned_params),
                                      cv::makePtr(search_params));
        this->flann_matcher = cv::makePtr(matcher);
    }
}

void FLANNMatcher::checkConfiguration(const int &flann_method) {
    if (flann_method < FLANN::KDTree || flann_method > FLANN::Autotuned) {
        throw std::invalid_argument("Flann method selected does not exist!");
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