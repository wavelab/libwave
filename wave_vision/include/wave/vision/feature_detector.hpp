#ifndef WAVE_FEATURE_DETECTOR_HPP
#define WAVE_FEATURE_DETECTOR_HPP

// C++ Headers
#include <exception>

// Libwave Headers
#include "wave/utils/utils.hpp"
#include "opencv_common.hpp"

namespace wave {

class FeatureDetectorConfigException : public std::exception {
    const char* what() const throw() {
        return "Failed to Load Detector Configuration";
    }
};

template <typename T>
class FeatureDetector {
 public:
    virtual ~FeatureDetector();

    Image& getImage() {
        return this->image;
    }

    virtual std::vector<Keypoint>& detectFeatures(Image& image_to_detect) = 0;

 protected:
    cv::Ptr<Image> image;
    std::vector<Keypoint> keypoints;
    virtual void loadImage(const T) = 0;
};

} // end of namespace wave

#endif //WAVE_FEATURE_DETECTOR_HPP
