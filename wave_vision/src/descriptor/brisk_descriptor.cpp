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
    std::vector<float> radiusList;
    radiusList[0] = f * 0.0f;
    radiusList[1] = f * 2.9f;
    radiusList[2] = f * 4.9f;
    radiusList[3] = f * 7.4f;
    radiusList[4] = f * 10.8f;

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

    this->brisk_descriptor =
      cv::BRISK::create(radiusList, numberList, dMax, dMin, indexChange);
}

// Destructor
BRISKDescriptor::~BRISKDescriptor() {}
}  // End of namespace wave