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

    // Create unused indexChange vector
    const std::vector<int> indexChange = std::vector<int>();

    // Create cv::BRISK object with the desired parameters
    this->brisk_descriptor = cv::BRISK::create(config.radiusList,
                                               config.numberList,
                                               config.dMax,
                                               config.dMin,
                                               indexChange);

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
    parser.addParam("radiusList", &config.radiusList);
    parser.addParam("numberList", &config.numberList);
    parser.addParam("dMax", &config.dMax);
    parser.addParam("dMin", &config.dMin);

    if (parser.load(config_path) != 0) {
        throw std::invalid_argument(
          "Failed to Load BRISKDescriptor Configuration");
    }

    // Create unused indexChange vector
    const std::vector<int> indexChange = std::vector<int>();

    // Confirm configuration is valid
    this->checkConfiguration(config);

    // Create cv::BRISK object with the desired parameters
    this->brisk_descriptor = cv::BRISK::create(config.radiusList,
                                               config.numberList,
                                               config.dMax,
                                               config.dMin,
                                               indexChange);

    // Store configuration parameters within member struct
    this->current_config = config;
}

BRISKDescriptor::~BRISKDescriptor() = default;

void BRISKDescriptor::checkConfiguration(
  const BRISKDescriptorParams &check_config) {
    std::vector<float> rList = check_config.radiusList;
    std::vector<int> nList = check_config.numberList;

    // Check that the size of radiusList and numberList are equal and positive
    if (rList.size() == 0) {
        throw std::invalid_argument("No parameters in radiusList!");
    } else if (nList.size() == 0) {
        throw std::invalid_argument("No parameters in numberList!");
    } else if (rList.size() != nList.size()) {
        throw std::invalid_argument(
          "radiusList and numberList are of unequal size!");
    }

    // Ensure all values of radiusList are positive
    std::vector<float>::iterator rListIterator;

    for (rListIterator = rList.begin(); rListIterator != rList.end();
         rListIterator++) {
        if (*rListIterator < 0) {
            throw std::invalid_argument("radiusList has a negative parameter!");
        }
    }

    // Ensure all values of numberList are positive
    std::vector<int>::iterator nListIterator;

    for (nListIterator = nList.begin(); nListIterator != nList.end();
         nListIterator++) {
        if (*nListIterator < 0) {
            throw std::invalid_argument("numberList has a negative parameter!");
        }
    }

    // Ensure dMax and dMin are both positive, and check dMax is less than dMin
    if (check_config.dMax < 0) {
        throw std::invalid_argument("dMax is a negative value!");
    } else if (check_config.dMin < 0) {
        throw std::invalid_argument("dMin is a negative value!");
    } else if (check_config.dMax > check_config.dMin) {
        throw std::invalid_argument("dMax is greater than dMin!");
    }
}

BRISKDescriptorParams BRISKDescriptor::getConfiguration() {
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
}  // end of namespace wave
