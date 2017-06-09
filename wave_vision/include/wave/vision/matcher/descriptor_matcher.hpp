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
    virtual ~DescriptorMatcher() = default;

    /** Virtual function to match keypoint descriptors between two images.
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

        cv::waitKey(0);
    }
};

}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP
