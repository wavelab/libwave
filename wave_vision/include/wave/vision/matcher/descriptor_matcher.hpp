/**
 * @file
 * Base class from which descriptor matchers can be derived.
 * @ingroup vision
 */
#ifndef WAVE_VISION_DESCRIPTOR_MATCHER_HPP
#define WAVE_VISION_DESCRIPTOR_MATCHER_HPP

/** C++ Headers */
#include <exception>

/** Third Party Headers */
#include <opencv2/opencv.hpp>

/** Libwave Headers */
#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Representation of a generic descriptor matcher.
 *
 *  Internally, this class, and all derived classes are wrapping various
 *  descriptor matchers implemented in OpenCV. Further reference on descriptor
 *  matchers can be found [here][opencv_descriptor_matchers].
 *
 *  [opencv_descriptor_matchers]:
 *  http://docs.opencv.org/trunk/db/d39/classcv_1_1DescriptorMatcher.html
 */
class DescriptorMatcher {
 public:
    /** Match keypoint descriptors between two images.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *
     *  @return vector containing the best matches.
     */
    virtual std::vector<cv::DMatch> matchDescriptors(
            cv::Mat &descriptors_1, cv::Mat &descriptors_2) = 0;



};

}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP