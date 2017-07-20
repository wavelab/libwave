/**
 * @file
 * Base class from which keypoint descriptors can be derived.
 * @ingroup vision
 */
#ifndef WAVE_VISION_DESCRIPTOR_EXTRACTOR_HPP
#define WAVE_VISION_DESCRIPTOR_EXTRACTOR_HPP

#include <exception>
#include <vector>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Representation of a generic keypoint descriptor extractor.
 *
 *  Internally, this class, and all derived classes are wrapping various
 *  descriptor extractors implemented in OpenCV. Further reference on Descriptor
 *  Extractors can be found [here][opencv_descriptor_extractors].
 *
 *  [opencv_descriptor_extractors]:
 *  http://docs.opencv.org/trunk/d5/d51/group__features2d__main.html
 */
class DescriptorExtractor {
 public:
    /** Extracts keypoint descriptors from an image. Calls a different extractor
     *  depending on the derived class.
     *
     *  @param image the image to extract keypoints from.
     *  @param keypoints the keypoints detected in the image.
     *
     *  @return descriptors, the computed keypoint descriptors.
     */
    virtual cv::Mat extractDescriptors(
      const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints) = 0;

 protected:
    /** Destructor */
    virtual ~DescriptorExtractor() = default;
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_EXTRACTOR_HPP
