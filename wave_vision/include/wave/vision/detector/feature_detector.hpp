/**
 * @file
 * Base class from which feature detectors can be derived.
 * @ingroup vision
 */
#ifndef WAVE_VISION_FEATURE_DETECTOR_HPP
#define WAVE_VISION_FEATURE_DETECTOR_HPP

#include <exception>
#include <vector>

#include <opencv2/opencv.hpp>

#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/**
 * Representation of a generic feature detector.
 *
 * Internally, this class, and all derived classes are wrapping various
 * detectors implemented in OpenCV. Further reference on OpenCV's Feature
 * Detectors can be found [here][opencv_feature_detectors].
 *
 * [opencv_feature_detectors]:
 * http://docs.opencv.org/trunk/d5/d51/group__features2d__main.html
 */
class FeatureDetector {
 public:
    virtual ~FeatureDetector() {}

    /** Function to return current image being detected
     *
     *  @return the image (in matrix form).
     */
    virtual cv::Mat &getImage() {
        return this->image;
    }

    /** Virtual function to detect features in an image. Calls a different
     *  detector depending on the derived class.
     *
     *  @param image, the image to detect features in.
     *  @return a vector of the detected keypoints.
     */
    virtual std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image) = 0;

 protected:
    /** The image currently being detected. */
    cv::Mat image;

    /** Function to set image within the member variables.
     *
     *  @param source_image, the image to load.
     */
    virtual void loadImage(const cv::Mat &source_image) {
        this->image = source_image;
    }
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_FEATURE_DETECTOR_HPP
