// Libwave Headers
#include "wave/vision/fast_detector.hpp"

namespace wave {

FASTDetector::FASTDetector(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int param_threshold;
    bool param_nonmax_suppression;
    int param_type;

    if (parser.load(config_path) != 0) {
        FeatureDetectorConfigException exception;
        throw exception;
    }

    parser.addParam("fast.threshold", &param_threshold);
    parser.addParam("fast.nonmax_suppression", &param_nonmax_suppression);
    parser.addParam("fast.type", &param_type);

    // Create FastFeatureDetector with these parameters
    this->fast_detector = cv::FastFeatureDetector::create();
    configureDetector(param_threshold, param_nonmax_suppression, param_type);

}

void FASTDetector::checkConfiguration(int new_threshold, int new_type) {
    // Check parameters. If invalid, throw an exception.
    try {
        if (new_threshold < 0) {
            std::cout << "Invalid threshold for FASTDetector!" << std::endl;
            throw "Exception";
        } else if (new_type < 1 || new_type > 3) {
            std::cout << "Invalid type for FASTDetector!" << std::endl;
            throw "Exception";
        }
    }
    catch (int) {
        std::cout << "Unhandled Exception" << std::endl;
    }
}

void FASTDetector::configureDetector(int new_threshold,
                                     bool new_nonmax_suppression,
                                     int new_type) {
    // Confirm configuration parameters are valid
    checkConfiguration(new_threshold, new_type);

    // Store configuration values in member variables
    this->threshold = new_threshold;
    this->nonmax_suppression = new_nonmax_suppression;
    this->type = new_type;

    // Set configuration values in detector.
    this->fast_detector->setThreshold(this->threshold);
    this->fast_detector->setNonmaxSuppression(this->nonmax_suppression);
    this->fast_detector->setType(this->type);
}

void FASTDetector::getConfiguration(int& current_threshold,
                                    bool& current_nonmax_suppression,
                                    int& current_type) {
    // Obtain current configuration values using cv::FastFeatureDetector::get**
    current_threshold = this->fast_detector->getThreshold();
    current_nonmax_suppression = this->fast_detector->getNonmaxSuppression();
    current_type = this->fast_detector->getType();
}

void FASTDetector::loadImage(Image& source_image) {
    this->image = source_image;
}

std::vector<Keypoint>& FASTDetector::detectFeatures(Image& image_to_detect) {
    // Load image
    this->loadImage(image_to_detect);

    // Detect features in image
    this->fast_detector->detect(this->image, this->keypoints);

    return this->keypoints;
}

} //end of namespace wave

