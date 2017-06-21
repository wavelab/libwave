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

    /** Filter matches using a heuristic based method.
     *
     *  If the distance between matches is less than the defined heuristic, it
     *  is rejected.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the matches with outliers removed.
     */
    virtual std::vector<cv::DMatch> filterMatches(
      std::vector<cv::DMatch> &matches) const = 0;

    /** Overloaded method, which takes in a vector of a vector of matches. This
     *  is designed to be used with the knnMatchDescriptors method, and uses the
     *  ratio test to filter the matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the filtered matches.
     */
    virtual std::vector<cv::DMatch> filterMatches(
      std::vector<std::vector<cv::DMatch>> &matches) const = 0;

    /** Remove outliers between matches by using epipolar constraints. Outlier
     *  rejection methods are specified within the MatcherParams struct.
     *
     *  @param matches the unfiltered matches computed from two images.
     *  @param keypoints_1 the keypoints detected in the first image.
     *  @param keypoints_2 they keypoints detected in the second image.
     *
     *  @return the matches with outliers removed.
     */
    virtual std::vector<cv::DMatch> removeOutliers(
      const std::vector<cv::DMatch> &matches,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2) const = 0;

 public:
    /** Matches keypoints descriptors between two images using the
     *  BruteForceMatcher.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *  @param keypoints_1 the keypoints detected in the first image
     *  @param keypoints_2 the keypoints detected in the second image
     *  @param mask The mask variable indicates which descriptors can be matched
     *  between the two sets. As per OpenCV docs "queryDescriptors[i] can be
     *  matched with trainDescriptors[j] only if masks.at<uchar>(i,j) is
     *  non-zero. In the libwave wrapper, queryDescriptors are descriptors_1,
     *  and trainDescriptors are descriptors_2.
     *
     *  @return vector containing the best matches.
     */
    virtual std::vector<cv::DMatch> matchDescriptors(
      const cv::Mat &descriptors_1,
      const cv::Mat &descriptors_2,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2,
      const cv::InputArray &mask = cv::noArray()) const = 0;
};
}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP
