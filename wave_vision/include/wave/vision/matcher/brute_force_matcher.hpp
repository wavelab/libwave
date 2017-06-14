/**
 * @file
 * Brute force matcher implementation, derived from descriptor matcher base.
 * @ingroup vision
 */
#ifndef WAVE_VISION_BRUTE_FORCE_MATCHER_HPP
#define WAVE_VISION_BRUTE_FORCE_MATCHER_HPP

/** C++ Headers */
#include <string>
#include <vector>

/** Libwave Headers */
#include "wave/utils/utils.hpp"
#include "wave/vision/matcher/descriptor_matcher.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

struct BFMatcherParams {
    /** Default constructor*/
    BFMatcherParams() {}

    /** Constructor using user-defined parameters */
    BFMatcherParams(int norm_type, bool cross_check)
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
     *  Recommended: true.
     */
    bool cross_check = true;


    /** Determines whether to use the ratio test or threshold-based distance
     *  heuristic for outlier rejection.
     *
     *  Recommended: true.
     */
    bool ratio_rejection = true;

    /** Specifies heuristic for the ratio test, illustrated by Dr. David G. Lowe
     *  in his paper _Distinctive Image Features from Scale-Invariant Keypoints_
     *  (2004). The test takes the ratio of the closest keypoint distance
     *  to that of the second closest neighbour. If the ratio is greater than
     * the
     *  heuristic, it is discarded.
     *
     *  A value of 0.8 was shown by Dr. Lowe to reject 90% of the false matches,
     *  and discard only 5% of the correct matches.
     *
     *  Recommended: 0.8. Must be between 0 and 1. Only used if @param
     *  ratio_rejection is true.
     */
    double ratio_test_heuristic = 0.8;

    /** Specifies the rejection heuristic for good matches.
     *
     *  Matches will only be kept if the descriptor distance is less than or
     *  equal to the product of the rejection heuristic and the _minimum_ of all
     *  descriptor distances. The greater the heuristic, the more matches will
     *  be kept.
     *
     *  Recommended: 5. Must be greater than or equal to zero. Only used if
     *  @param ratio_rejection is false.
     */
    int rejection_heuristic = 0;

    /**
     *
     * Options:
     * cv::RANSAC: Random Sample Consensus estimation
     * cv::LMEDS:
     */
    // int estimator = cv::RANSAC;
};

class BruteForceMatcher : public DescriptorMatcher {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation. Uses default values if not specified.
     */
    explicit BruteForceMatcher(
      const BFMatcherParams &config = BFMatcherParams{});

    /** Constructs a BruteForceMatcher using parameters found in the
     *  linked .yaml file.
     *
     *  @param config_path is the path to a .yaml file, containing the desired
     *  parameters for the BruteForceMatcher.
     */
    BruteForceMatcher(const std::string &config_path);

    /** Returns the current configuration parameters being used by the
     *  DescriptorMatcher.
     *
     * @return a struct containing the current configuration values.
     */
    BFMatcherParams getConfiguration() const {
        return this->current_config;
    }

    /** Matches keypoints descriptors between two images using the
     *  BruteForceMatcher.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *
     *  @return vector containing the best matches.
     */
    std::vector<cv::DMatch> matchDescriptors(
      cv::Mat &descriptors_1, cv::Mat &descriptors_2) const override;

    /** Remove outliers between matches. Uses a heuristic based approach as a
     *  first pass to determine good matches. Determines the fundamental
     *  matrix to act as a mask for further refinement.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the matches with outliers removed.
     */
    std::vector<cv::DMatch> removeOutliers(
      std::vector<cv::DMatch> matches) const override;

    /** Overloaded method, which takes in a vector of a vector of matches. This
     *  is designed to be used with the knnMatchDescriptors method, and uses the
     *  ratio test as a first pass to determine good matches. Determines the
     *  fundamental matrix to act as a mask for further refinement.
     *
     *  @param matches the unfiltered matches computed from two images.
     *
     *  @return the matches with outliers removed.
     */
    std::vector<cv::DMatch> removeOutliers(
      std::vector<std::vector<cv::DMatch>> matches) const override;

 private:
    /** The pointer to the wrapped cv::BFMatcher object */
    cv::Ptr<cv::BFMatcher> brute_force_matcher;

    /** Current configuration parameters */
    BFMatcherParams current_config;

    /** Checks whether the desired configuration is valid
     *
     *  @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const BFMatcherParams &check_config);
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_VISION_BRUTE_FORCE_MATCHER_HPP
