/**
 * @file
 * Base class from which descriptor matchers can be derived.
 * @ingroup vision
 */
#ifndef WAVE_VISION_DESCRIPTOR_MATCHER_HPP
#define WAVE_VISION_DESCRIPTOR_MATCHER_HPP

#include <exception>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Parameters for calculating the Fundamental Matrix, when removing outliers
 * from the matches.
 */
struct FMParams {
    /// Default constructor.
    FMParams() = default;

    /** Overloaded constructor.
     *
     * @param max_dist The desired maximum distance from a point to an epipolar
     * line. Cannot be below 0.
     * @param confidence The desired confidence interval of the estimated
     * fundamental matrix. Must be between 0 and 1.
     * @throw std::invalid_argument if either parameter is out of range.
     */
    FMParams(double max_dist, double confidence)
        : max_dist{max_dist}, confidence{confidence} {
        if (this->max_dist < 0) {
            throw std::invalid_argument("max_dist must be greater than 0!");
        }

        if (this->confidence < 0 || this->confidence > 1) {
            throw std::invalid_argument("confidence must be between 0 and 1!");
        }
    }

    /** Maximum distance from a point to an epipolar line, in pixels. Any points
     * further are considered outliers. Only used for RANSAC.
     *
     * Recommended: 1.0. Cannot be less than 0.
     */
    double max_dist = 1.0;

    /** Desired confidence interval of the estimated fundamental matrix. Only
     * used for RANSAC or LMedS methods.
     *
     * Recommended: 0.99. Must be between 0 and 1.
     */
    double confidence = 0.99;
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
    /** Remove outliers between matches by using epipolar constraints. Outlier
     *  rejection methods are specified within the respective MatcherParams
     *  struct.
     *
     *  @param matches the unfiltered matches computed from two images.
     *  @param keypoints_1 the keypoints detected in the first image.
     *  @param keypoints_2 the keypoints detected in the second image.
     *
     *  @return the matches with outliers removed.
     */
    virtual std::vector<cv::DMatch> removeOutliers(
      const std::vector<cv::DMatch> &matches,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2) const = 0;

    /** Matches keypoints descriptors between two images using the
     *  BruteForceMatcher.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *  @param keypoints_1 the keypoints detected in the first image
     *  @param keypoints_2 the keypoints detected in the second image
     *  @param mask
     *  \parblock indicates which descriptors can be matched between the two
     *  sets. As per OpenCV docs "queryDescriptors[i] can be matched with
     *  trainDescriptors[j] only if masks.at<uchar>(i,j) is non-zero. In the
     *  libwave wrapper, queryDescriptors are descriptors_1, and
     *  trainDescriptors are descriptors_2. Default is cv::noArray().
     *  \endparblock
     *
     *  @return vector containing the best matches.
     */
    virtual std::vector<cv::DMatch> matchDescriptors(
      cv::Mat &descriptors_1,
      cv::Mat &descriptors_2,
      const std::vector<cv::KeyPoint> &keypoints_1,
      const std::vector<cv::KeyPoint> &keypoints_2,
      cv::InputArray mask) = 0;

    /** Returns the number of raw, filtered, and good matches from the latest
     * set of images.
     *
     * @param[out] num_raw_matches The number of matches prior to any filtering
     * or outlier removal.
     * @param[out] num_filtered_matches The number of matches after being
     * filtered by the distance/ratio test.
     * @param[out] num_good_matches The number of matches remaining after
     * outlier removal via RANSAC or a similar method
     */
    void getNumMatches(uint64_t &num_raw_matches,
                       uint64_t &num_filtered_matches,
                       uint64_t &num_good_matches) {
        num_raw_matches = this->num_raw_matches;
        num_filtered_matches = this->num_filtered_matches;
        num_good_matches = this->num_good_matches;
    }

 protected:
    virtual ~DescriptorMatcher() = default;

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
      const std::vector<cv::DMatch> &matches) const = 0;

    /** Overloaded method, which takes in a vector of a vector of matches. This
     *  is designed to be used with the knnMatchDescriptors method, and uses the
     *  ratio test to filter the matches.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the filtered matches.
     */
    virtual std::vector<cv::DMatch> filterMatches(
      const std::vector<std::vector<cv::DMatch>> &matches) const = 0;

 protected:
    /** The number of matches prior to any filtering or outlier removal. */
    uint64_t num_raw_matches = 0u;

    /** The number of matches after being filtered by the distance/ratio test.
     *  This is the input to the outlierRejection method.
     */
    uint64_t num_filtered_matches = 0u;

    /** The number of matches remaining after outlier removal via RANSAC or a
     *  similar method.
     */
    uint64_t num_good_matches = 0u;
};
}  // namespace wave

#endif  // WAVE_VISION_DESCRIPTOR_MATCHER_HPP
