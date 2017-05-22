#ifndef WAVE_FEATURE_DETECTOR_HPP
#define WAVE_FEATURE_DETECTOR_HPP

// C++ Headers
#include <exception>

// Libwave Headers
#include "wave/utils/utils.hpp"

// Third Party Headers
#include "opencv2/opencv.hpp"

namespace wave {

class ConfigurationLoadingException : public std::exception {
    const char* what() const throw() {
        return "Failed to Load Detector Configuration";
    }
};

class InvalidConfigurationException : public std::exception {
    const char* what() const throw() {
        return "Invalid Detector Configuration";
    }
};

template <typename T>
class FeatureDetector {
 public:
    virtual ~FeatureDetector() {};

    virtual T& getImage() {
        return this->image;
    }

    virtual std::vector<cv::KeyPoint>& detectFeatures(const T& image) = 0;

 protected:
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;

    virtual void loadImage(const T& source_image) {
        this->image = source_image;
    };
};

} // end of namespace wave

#endif //WAVE_FEATURE_DETECTOR_HPP
