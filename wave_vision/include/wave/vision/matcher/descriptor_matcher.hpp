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
 protected:
    /** Destructor */
    ~DescriptorMatcher() = default;

 public:
    /** Match keypoint descriptors between two images.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *
     *  @return vector containing the best matches.
     */
    virtual std::vector<cv::DMatch> matchDescriptors(
      const cv::Mat &descriptors_1, const cv::Mat &descriptors_2) const = 0;

    /** Remove outliers between matches using various outlier rejection methods.
     *  Outlier rejection methods are specified within the MatcherParams struct.
     *  Uses a heuristic based approach as a first pass to determine good
     *  matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the matches with outliers removed.
     */
    virtual std::vector<cv::DMatch> removeOutliers(
      std::vector<cv::DMatch> &matches) = 0;

    /** Overloaded method, which takes in a vector of a vector of matches. This
     *  is designed to be used with the knnMatchDescriptors method, and uses the
     *  ratio test as a first pass to determine good matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the matches with outliers removed.
     */
    virtual std::vector<cv::DMatch> removeOutliers(
      std::vector<std::vector<cv::DMatch>> &matches) = 0;
};

}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP