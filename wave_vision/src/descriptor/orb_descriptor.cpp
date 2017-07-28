#include "wave/vision/descriptor/orb_descriptor.hpp"

namespace wave {

// Filesystem based constructor for ORBDescriptorParams
ORBDescriptorParams::ORBDescriptorParams(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int wta_k;
    int patch_size;

    // Add parameters to parser, to be loaded. If path cannot be found, throw
    // an exception
    parser.addParam("wta_k", &wta_k);
    parser.addParam("patch_size", &patch_size);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to Load ORBDescriptorParams Configuration");
    }

    this->wta_k = wta_k;
    this->patch_size = patch_size;
}

// Default constructor. Struct may be default or user defined.
ORBDescriptor::ORBDescriptor(const ORBDescriptorParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Default parameters that should not be modified, and are not associated
    // with the descriptor. These values are the defaults recommended by OpenCV.
    //-------------------------------------------------------------------------
    int num_features = 500;
    float scale_factor = 1.2f;
    int num_levels = 8;
    int edge_threshold = 31;
    int first_level = 0;  // As per OpenCV docs, first_level must be zero.
    int score_type = cv::ORB::HARRIS_SCORE;
    int fast_threshold = 20;

    // Create cv::ORB object with the desired parameters
    this->orb_descriptor = cv::ORB::create(num_features,
                                           scale_factor,
                                           num_levels,
                                           edge_threshold,
                                           first_level,
                                           config.wta_k,
                                           score_type,
                                           config.patch_size,
                                           fast_threshold);
}

void ORBDescriptor::checkConfiguration(
  const ORBDescriptorParams &check_config) {
    // Check that the value of wta_k is between 2 and 4, and that patch_size
    // is greater than zero.
    if (check_config.wta_k < 2 || check_config.wta_k > 4) {
        throw std::invalid_argument("wta_k is not an acceptable value!");
    } else if (check_config.patch_size <= 0) {
        throw std::invalid_argument("patch_size is less than/ equal to zero!");
    }
}

void ORBDescriptor::configure(const ORBDescriptorParams &new_config) {
    this->checkConfiguration(new_config);

    // Configure orb_descriptor using cv::ORB::set**
    this->orb_descriptor->setWTA_K(new_config.wta_k);
    this->orb_descriptor->setPatchSize(new_config.patch_size);
}

ORBDescriptorParams ORBDescriptor::getConfiguration() const {
    // Obtain current configuration values using cv::ORB::get**
    auto curr_wta_k = this->orb_descriptor->getWTA_K();
    auto curr_patch_size = this->orb_descriptor->getPatchSize();

    ORBDescriptorParams current_config{curr_wta_k, curr_patch_size};

    return current_config;
}

cv::Mat ORBDescriptor::extractDescriptors(
  const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints) {
    cv::Mat descriptors;

    this->orb_descriptor->compute(image, keypoints, descriptors);

    return descriptors;
}
}  // namespace wave
