// Libwave Headers
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

// Default constructor. Struct may be default or user defined.
BruteForceMatcher::BruteForceMatcher(const MatcherParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Create cv::BFMatcher object with the desired parameters
    this->brute_force_matcher =
      cv::BFMatcher::create(config.norm_type, config.cross_check);

    // Store configuration parameters within member struct
    this->current_config = config;
}

BruteForceMatcher::BruteForceMatcher(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    // Configuration parameters
    auto config = MatcherParams{};

    // Add parameters to parser, to be loaded. If path cannot be found, throw an
    // exception.
    parser.addParam("norm_type", &config.norm_type);
    parser.addParam("cross_check", &config.cross_check);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to load BruteForceMatcher configuration! Path not found.");
    }

    // Confirm configuration is valid
    this->checkConfiguration(config);

    // Create cv::BFMatcher object with the desired parameters
    this->brute_force_matcher =
      cv::BFMatcher::create(config.norm_type, config.cross_check);

    // Store configuration parameters within member struct
    this->current_config = config;
}

void BruteForceMatcher::checkConfiguration(const MatcherParams &check_config) {
    // Check that the value of norm_type is one of the valid values
    if (check_config.norm_type < cv::NORM_INF ||
        check_config.norm_type > cv::NORM_HAMMING2) {
        throw std::invalid_argument(
          "Norm type is not one of the acceptable values!");
    }
}

std::vector<cv::DMatch> BruteForceMatcher::matchDescriptors(
  cv::Mat &descriptors_1, cv::Mat &descriptors_2) {
    std::vector<cv::DMatch> matches;

    return matches;
}


}  // namespace wave