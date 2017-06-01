/**
 * @file
 * Base class from which descriptor matchers can be derived.
 * @ingroup vision
 */
#ifndef WAVE_DESCRIPTOR_MATCHER_HPP
#define WAVE_DESCRIPTOR_MATCHER_HPP

/** C++ Headers */
#include <exception>

/** Third Party Headers */
#include <opencv2/opencv.hpp>

/** Libwave Headers */
#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Enum to determine matcher type:
 *
 *  base matcher determines the best keypoint matches across the query set.
 *  k-nearest-neighbours finds the k best matches for each descriptor.
 *  radius threshold returns the best keypoint matches within a certain
 *  distance.
 */
enum matcherType { base, knn, radius };

/** Struct for full matcher configuration.
 *
 *  base matcher determines the best keypoint matches across the query set.
 *  k-nearest-neighbours finds the k best matches for each descriptor.
 *  radius threshold returns the best keypoint matches within a certain
 *  distance.
 */
struct MatcherConfig {
    /** Enum to determine matcher type. Default: base */
    matcherType type = base;
};

/** Representation of a generic descriptor matcher.
 *
 *  Internally, this class, and all derived classes are wrapping various
 *  descriptor matchers implemented in OpenCV. Further reference on descriptor
 *  matchers can be found [here][opencv_descriptor_matchers].
 *
 *  [opencv_descriptor_matchers]:
 * http://docs.opencv.org/trunk/db/d39/classcv_1_1DescriptorMatcher.html
 */
class DescriptorMatcher {
 public:
    /** Descructor */
    virtual ~DescriptorMatcher() = default;

    /** Virtual function to match keypoints descriptors between two images.
     *  Templated in order to accommodate return of base, knn, or radius
     *  matchers.
     *
     *  @param descriptors_1 the descriptors extracted from the first image.
     *  @param descriptors_2 the descriptors extracted from the second image.
     *  @param matches either std::vector<cv::DMatch>, or
     *  std::vector<std::vector<cv::DMatch>> depending on selection of base,
     *  knn, or radius matchers.
     */
    template <typename T>
    void matchDescriptors(cv::Mat descriptors_1,
                          cv::Mat descriptors_2,
                          T matches) = 0;

 private:
    MatcherConfig config;
};


}  // namespace wave

#endif  // WAVE_DESCRIPTOR_MATCHER_HPP
