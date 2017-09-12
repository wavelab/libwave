/**
 * @file
 * Base class from which feature detectors can be derived.
 * @ingroup vision
 */
#ifndef WAVE_VISION_FEATURE_DETECTOR_HPP
#define WAVE_VISION_FEATURE_DETECTOR_HPP

#include <exception>
#include <vector>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Representation of a generic feature detector.
 *
 *  Internally, this class, and all derived classes are wrapping various
 *  detectors implemented in OpenCV. Further reference on OpenCV's Feature
 *  Detectors can be found [here][opencv_feature_detectors].
 *
 *  [opencv_feature_detectors]:
 *  http://docs.opencv.org/trunk/d5/d51/group__features2d__main.html
 */
class FeatureDetector {
 public:
    /** Detects features in an image. Calls a different detector depending on
     *  the derived class.
     *
     *  @param image the image to detect features in.
     *
     *  @return a vector of the detected keypoints.
     */
    virtual std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image) = 0;

 protected:
    virtual ~FeatureDetector() = default;
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_FEATURE_DETECTOR_HPP
