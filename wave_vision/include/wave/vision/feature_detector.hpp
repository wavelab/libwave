#ifndef WAVE_FEATURE_DETECTOR_HPP
#define WAVE_FEATURE_DETECTOR_HPP

// C++ Headers
#include <exception>

// Libwave Headers
#include "wave/utils/utils.hpp"
#include "opencv_common.hpp"

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

// Templates for image type and keypoints.
template <typename T>

class FeatureDetector {
 public:
    virtual ~FeatureDetector();

    virtual cv::Ptr<T> getImage() {
        return this->image;
    }

    virtual std::vector<Keypoint>& detectFeatures(T& image_to_detect) = 0;

 protected:
    cv::Ptr<T> image;
    std::vector<Keypoint> keypoints;

    virtual void loadImage(T& source_image) {
        this->image = &source_image;
    };
};

} // end of namespace wave

#endif //WAVE_FEATURE_DETECTOR_HPP
