// Libwave Headers
#include "wave/vision/matcher/brute_force_matcher.hpp"

namespace wave {

// Default constructor. Struct may be default or user defined.
BruteForceMatcher::BruteForceMatcher(const BFMatcherParams &config) {
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
    auto config = BFMatcherParams{};

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

void BruteForceMatcher::checkConfiguration(
  const BFMatcherParams &check_config) {
    // Check that the value of norm_type is one of the valid values
    if (check_config.norm_type < cv::NORM_INF ||
        check_config.norm_type > cv::NORM_HAMMING2 ||
        check_config.norm_type == 3) {
        throw std::invalid_argument(
          "Norm type is not one of the acceptable values!");
    }
}

std::vector<cv::DMatch> BruteForceMatcher::matchDescriptors(
  const cv::Mat &descriptors_1, const cv::Mat &descriptors_2) const {
    std::vector<cv::DMatch> matches;

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

    // Determine matches between sets of descriptors
    this->brute_force_matcher->match(
      descriptors_1, descriptors_2, matches, mask);

    return matches;
}

}  // namespace wave
