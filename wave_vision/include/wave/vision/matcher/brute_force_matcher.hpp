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

class BruteForceMatcher : DescriptorMatcher {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation. Uses default values if not specified.
     */
    explicit BruteForceMatcher(const MatcherParams &config = MatcherParams{});

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
     *  BruteForceMatcher.
     *
     * @return a struct containing the current configuration values.
     */
    MatcherParams getConfiguration() const;

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

    /** The current configuration*/
    MatcherParams current_config;

    /** Checks whether the desired configuration is valid
     *
     *  @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const MatcherParams &check_config);
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_VISION_BRUTE_FORCE_MATCHER_HPP
