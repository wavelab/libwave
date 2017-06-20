// Libwave Headers
#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

// Default Constructor
FASTDetector::FASTDetector(const FASTParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Create FastFeatureDetector with these parameters
    this->fast_detector = cv::FastFeatureDetector::create(
      config.threshold, config.nonmax_suppression, config.type);
}

// Constructor using .yaml file, located at config_path.
FASTDetector::FASTDetector(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    // Configuration parameters
    FASTParams config;

    // Add parameters to parser, to be loaded. If path cannot be found, throw an
    // exception.
    parser.addParam("threshold", &config.threshold);
    parser.addParam("nonmax_suppression", &config.nonmax_suppression);
    parser.addParam("type", &config.type);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to Load FASTDetector Configuration");
    }

    // Verify configuration values
    this->checkConfiguration(config);

    // Create FastFeatureDetector with these parameters
    this->fast_detector = cv::FastFeatureDetector::create(
      config.threshold, config.nonmax_suppression, config.type);
}

void FASTDetector::checkConfiguration(const FASTParams &check_config) {
    // Check parameters. If invalid, throw an exception.
    if (check_config.threshold < 0) {
        throw std::invalid_argument("Invalid threshold for FASTDetector!");
    } else if (check_config.type < 0 || check_config.type > 3) {
        throw std::invalid_argument("Invalid type for FASTDetector!");
    }
}

void FASTDetector::configure(const FASTParams &new_config) {
    // Confirm configuration parameters are valid
    this->checkConfiguration(new_config);

    // Set configuration values in detector.
    this->fast_detector->setThreshold(new_config.threshold);
    this->fast_detector->setNonmaxSuppression(new_config.nonmax_suppression);
    this->fast_detector->setType(new_config.type);
}

FASTParams FASTDetector::getConfiguration() const {
    FASTParams current_config;

    // Obtain current configuration values using cv::FastFeatureDetector::get**
    current_config.threshold = this->fast_detector->getThreshold();
    current_config.nonmax_suppression =
      this->fast_detector->getNonmaxSuppression();
    current_config.type = this->fast_detector->getType();

    return current_config;
}

std::vector<cv::KeyPoint> FASTDetector::detectFeatures(const cv::Mat &image) {
    std::vector<cv::KeyPoint> keypoints;

    // Detect features in image, save values into keypoints.
    this->fast_detector->detect(image, keypoints);

    return keypoints;
}

}  // namespace wave
