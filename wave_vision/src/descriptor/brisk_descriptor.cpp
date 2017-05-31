// Libwave Headers
#include "wave/vision/descriptor/brisk_descriptor.hpp"

namespace wave {

// Default constructor
BRISKDescriptor::BRISKDescriptor() {
    // Instantiate cv::BRISK with default values
    float patternScale = 1.0f;
    float f = 0.85f * patternScale;

    // radiusList contains the radius (in pixels) of each circle in the sampling
    // pattern
    std::vector<float> radiusList = {
      f * 0.0f, f * 2.9f, f * 4.9f, f * 7.4f, f * 10.8f};

    // numberList contains the number of points for each subsequent circle in
    // the pattern
    std::vector<int> numberList = {1, 10, 14, 15, 20};

    // Threshold distances to classify pairs of points. dMax is maximum for
    // short pairs, dMin is the minimum for long pairs.
    float dMax = 5.85f;
    float dMin = 8.2f;

    /** OpenCV refers to this as a parameter for "index remapping of the bits."
     *  Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
     *  with the OpenCV Library" states this parameter is unused, and should be
     *  omitted.
     */
    const std::vector<int> indexChange = std::vector<int>();

    // Create cv::BRISK object
    this->brisk_descriptor =
      cv::BRISK::create(radiusList, numberList, dMax, dMin, indexChange);

    // Store configuration parameters within member struct
    this->current_config =
      BRISKDescriptorParams{radiusList, numberList, dMax, dMin};
}

// Constructor using BRISKDescriptorParams struct
BRISKDescriptor::BRISKDescriptor(const BRISKDescriptorParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    /** OpenCV refers to this as a parameter for "index remapping of the bits."
     *  Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
     *  with the OpenCV Library" states this parameter is unused, and should be
     *  omitted.
     */
    const std::vector<int> index_change = std::vector<int>();

    // Create cv::BRISK object with the desired parameters
    this->brisk_descriptor = cv::BRISK::create(config.radius_list,
                                               config.number_list,
                                               config.d_max,
                                               config.d_min,
                                               index_change);

    // Store configuration parameters within member struct
    this->current_config = config;
}

BRISKDescriptor::BRISKDescriptor(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    // Configuration parameters
    BRISKDescriptorParams config;

    // Add parameters to parser, to be loaded. If path cannot be found, throw an
    // exception.
    parser.addParam("radius_list", &config.radius_list);
    parser.addParam("number_list", &config.number_list);
    parser.addParam("d_max", &config.d_max);
    parser.addParam("d_min", &config.d_min);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to Load BRISKDescriptor Configuration");
    }

    // Confirm configuration is valid
    this->checkConfiguration(config);

    /** OpenCV refers to this as a parameter for "index remapping of the bits."
     *  Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
     *  with the OpenCV Library" states this parameter is unused, and should be
     *  omitted.
     */
    const std::vector<int> index_change = std::vector<int>();

    // Create cv::BRISK object with the desired parameters
    this->brisk_descriptor = cv::BRISK::create(config.radius_list,
                                               config.number_list,
                                               config.d_max,
                                               config.d_min,
                                               index_change);

    // Store configuration parameters within member struct
    this->current_config = config;
}

BRISKDescriptor::~BRISKDescriptor() = default;

void BRISKDescriptor::checkConfiguration(
  const BRISKDescriptorParams &check_config) {
    std::vector<float> rlist = check_config.radius_list;
    std::vector<int> nlist = check_config.number_list;

    // Check that the size of radiusList and numberList are equal and positive
    if (rlist.size() == 0) {
        throw std::invalid_argument("No parameters in radius_list!");
    } else if (nlist.size() == 0) {
        throw std::invalid_argument("No parameters in number_list!");
    } else if (rlist.size() != nlist.size()) {
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

    // Call compute method, and return descriptors
    this->loadImage(image);
    this->brisk_descriptor->compute(image, keypoints, descriptors);

    return descriptors;
}
}  // namespace wave
