/**
 * @file
 * Base class from which keypoint descriptors can be derived.
 * @ingroup vision
 */
#ifndef WAVE_DESCRIPTOR_EXTRACTOR_HPP
#define WAVE_DESCRIPTOR_EXTRACTOR_HPP

/** C++ Headers */
#include <exception>
#include <vector>

/** Third Party Headers */
#include <opencv2/opencv.hpp>

/** Libwave Headers */
#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/**
 *  Representation of a generic keypoint descriptor extractor.
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
    /** Destructor */
    virtual ~DescriptorExtractor() = default;

    /** Function to return the current image being detected
     *
     * @return the image (in matrix form).
     */
    virtual cv::Mat &getimage() {
        return this->image;
    }

    /** Virtual function to extract keypoint descriptors from an image. Calls a
     *  different extractor depending on the derived class.
     *
     *  @param image, the image to extract keypoints from.
     *  @param keypoints, the keypoints detected in the image.
     *
     *  @return descriptors, the computed keypoint descriptors.
     */
    virtual cv::Mat extractDescriptors(
      cv::Mat &image, std::vector<cv::KeyPoint> &keypoints) = 0;

 protected:
    /** The image from which to extract descriptors */
    cv::Mat image;

    /** Function to set image within the member variable
     *
     *  @param source_image, the image to load
     */
    virtual void loadImage(const cv::Mat &source_image) {
        this->image = source_image;
    };
};
}  // namespace wave

#endif  // WAVE_DESCRIPTOR_EXTRACTOR_HPP
