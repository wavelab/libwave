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

struct MatcherParams {
    /** Default constructor*/
    MatcherParams() {}

    /** Constructor using user-defined parameters */
    MatcherParams(int norm_type, bool cross_check)
        : norm_type(norm_type), cross_check(cross_check) {}

    /** Norm type to use for distance calculation between feature descriptors.
     *
     *  Options:
     *  cv::NORM_INF: l-infinity norm
     *  cv::NORM_L1: l1 norm
     *  cv::NORM_L2: l2 norm
     *  cv::NORM_L2SQR: l2 norm, squared
     *  cv::NORM_HAMMING: Hamming distance
     *  cv::NORM_HAMMING2:
     *
     *  As per OpenCV docs, NORM_L1 and NORM_L2 is valid for the SIFT or
     *  SURF descriptors, while NORM_HAMMING is valid for the ORB, BRISK, and
     *  BRIEF descriptors. NORM_HAMMING2 should only be used with ORB, when
     *  WTA_K is 3 or 4.
     *
     *  Default: NORM_HAMMING, since currently only BRISK Descriptor is used.
     *
     *  Please refer to further description of the norm types
     *  [here][opencv_norm_types].
     *
     *  [opencv_norm_types]:
     *  http://docs.opencv.org/trunk/d2/de8/group__core__array.html#gad12cefbcb5291cf958a85b4b67b6149f
     */
    int norm_type = cv::NORM_HAMMING;

    /** This boolean enables or disables the cross-checking functionality, which
     *  looks to remove false matches. If true, the closest match to
     *  descriptor_1 from descriptor_2 will only be reported if the object in
     *  descriptor_2 is also the closest to descriptor_1.
     *
     *  Recommended: false
     */
    bool cross_check = false;
};

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
    /** Destructor */
    virtual ~DescriptorMatcher() = default;

    /** Virtual function to match keypoints descriptors between two images.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *
     *  @return vector containing the best matches.
     */
    virtual std::vector<cv::DMatch> matchDescriptors(
      cv::Mat &descriptors_1, cv::Mat &descriptors_2) = 0;

    /** Function to display matches between two images.
     *
     *  @param image_1 the first image.
     *  @param keypoints_1 the keypoints detected in the first image.
     *  @param image_2 the second image.
     *  @param keypoints_2 the keypoints detected in the second image.
     *  @param matches the matches found between the images.
     */
    void showMatches(cv::Mat image_1,
                     std::vector<cv::KeyPoint> keypoints_1,
                     cv::Mat image_2,
                     std::vector<cv::KeyPoint> keypoints_2,
                     std::vector<cv::DMatch> matches) {
        cv::Mat image_matches;

        cv::drawMatches(
          image_1, keypoints_1, image_2, keypoints_2, matches, image_matches);

        cv::imshow("Matched Keypoints", image_matches);
    }

 protected:
    /** Protected mask variable, currently unused.
     *
     *  The mask variable indicates which descriptors can be matched between the
     *  two sets. As per OpenCV docs "queryDescriptors[i] can be matched with
     *  trainDescriptors[j] only if masks.at<uchar>(i,j) is non-zero.
     *
     *  In the libwave wrapper, queryDescriptors and trainDescriptors are
     *  referred to as descriptors_1 and descriptors_2.
     */
    cv::InputArray mask = cv::noArray();
};

}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP
