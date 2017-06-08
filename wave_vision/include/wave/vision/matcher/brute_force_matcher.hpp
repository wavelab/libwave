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
     *  Recommended: false
     */
    bool cross_check = false;
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

    /** Destructor */
    ~BruteForceMatcher() = default;

    /** Returns the current configuration parameters being used by the
     *  DescriptorMatcher.
     *
     * @return a struct containing the current configuration values.
     */
    BFMatcherParams getConfiguration() const {
        return this->current_config;
    }

    /** Function to match keypoints descriptors between two images using the
     *  BruteForceMatcher.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *
     *  @return vector containing the best matches.
     */
    std::vector<cv::DMatch> matchDescriptors(cv::Mat &descriptors_1,
                                             cv::Mat &descriptors_2);

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
