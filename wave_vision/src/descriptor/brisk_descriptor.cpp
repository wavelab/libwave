// Libwave Headers
#include "wave/vision/descriptor/brisk_descriptor.hpp"

namespace wave {

// Default constructor. Struct may be default or user defined.
BRISKDescriptor::BRISKDescriptor(const BRISKDescriptorParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Create cv::BRISK object with the desired parameters
    this->brisk_descriptor = cv::BRISK::create(config.radius_list,
                                               config.number_list,
                                               config.d_max,
                                               config.d_min,
                                               config.index_change);

    // Store configuration parameters within member struct
    this->current_config = config;
}

void BRISKDescriptor::checkConfiguration(
  const BRISKDescriptorParams &check_config) {
    // Check that the size of radiusList and numberList are equal and positive
    if (check_config.radius_list.size() == 0) {
        throw std::invalid_argument("No parameters in radius_list!");
    } else if (check_config.number_list.size() == 0) {
        throw std::invalid_argument("No parameters in number_list!");
    } else if (check_config.radius_list.size() !=
               check_config.number_list.size()) {
        throw std::invalid_argument(
          "radius_list and number_list are of unequal size!");
    }

    // Ensure all values of radiusList are positive
    for (const auto &radius : check_config.radius_list) {
        if (radius < 0) {
            throw std::invalid_argument(
              "radius_list has a negative parameter!");
        }
    }

    // Ensure all values of numberList are positive
    for (auto &num_points : check_config.number_list) {
        if (num_points < 0) {
            throw std::invalid_argument(
              "number_list has a negative parameter!");
        }
    }

    // Ensure dMax and dMin are both positive, and check dMax is less than dMin
    if (check_config.d_max < 0) {
        throw std::invalid_argument("d_max is a negative value!");
    } else if (check_config.d_min < 0) {
        throw std::invalid_argument("d_min is a negative value!");
    } else if (check_config.d_max > check_config.d_min) {
        throw std::invalid_argument("d_max is greater than d_min!");
    }
}

BRISKDescriptorParams BRISKDescriptor::getConfiguration() const {
    return this->current_config;
}

cv::Mat BRISKDescriptor::extractDescriptors(
  cv::Mat &image, std::vector<cv::KeyPoint> &keypoints) {
    cv::Mat descriptors;

    this->brisk_descriptor->compute(image, keypoints, descriptors);

    return descriptors;
}
}  // namespace wave
