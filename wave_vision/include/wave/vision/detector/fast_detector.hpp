/**
 * @file
 * FAST Feature Detector implementation, derived from Feature Detector base
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_FAST_DETECTOR_HPP
#define WAVE_VISION_FAST_DETECTOR_HPP

#include <string>
#include <vector>

#include "wave/vision/detector/feature_detector.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Contains the configuration parameters for the FASTDetector. */
struct FASTParams {
    /** Default Constructor */
    FASTParams() {}

    FASTParams(int threshold, bool nonmax_suppression, int type)
        : threshold(threshold),
          nonmax_suppression(nonmax_suppression),
          type(type) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     *  @param config_path the path to the location of the configuration file
     */
    FASTParams(const std::string &config_path);

    /** Threshold on difference between intensity of the central pixel, and
     *  pixels in a circle (Bresenham radius 3) around this pixel. Must be
     *  greater than zero.
     *
     *  Recommended: 10
     */
    int threshold = 10;

    /** Removes keypoints in adjacent locations.
     *
     *  Recommended: true
     */
    bool nonmax_suppression = true;

    /** Neighbourhood, as defined in the paper by Rosten. TYPE_N_M refers to
     *  the pixel circumference (M) and number of consecutive pixels (N) that
     *  must be brighter or darker than the center pixel for the algorithm to
     *  deem the point as a corner.
     *
     *  Options:
     *  cv::FastFeatureDetector::TYPE_5_8
     *  cv::FastFeatureDetector::TYPE_7_12
     *  cv::FastFeatureDetector::TYPE_9_16 (recommended)
     */
    int type = cv::FastFeatureDetector::TYPE_9_16;
};

/** Representation of a feature detector using the FAST algorithm.
 *
 *  Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 *  Further reference on the FastFeatureDetector can be found
 *  [here][opencv_feature_detectors].
 *
 *  [opencv_feature_detectors]:
 *  http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public FeatureDetector {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values
     */
    FASTDetector(const FASTParams &config = FASTParams{});

    /** Reconfigures the FastFeatureDetector object with new values requested by
     *  the user
     *
     * @param new_config containing the desired configuration values.
     */
    void configure(const FASTParams &new_config);

    /** Returns the current configuration parameters being used by the
     *  FastFeatureDetector.
     *
     * @return a struct containing the current configuration values.
     */
    FASTParams getConfiguration() const;

    /** Detects features in an image.
     *
     *  @param image the image to detect features in.
     *  @return a vector containing all of the keypoints found within the image.
     */
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image);

 private:
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

    /** Checks whether the desired configuration is valid.
     *
     *  The threshold value must be greater than zero, while the type must be
     *  0, 1, or 2.
     *
     *  @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const FASTParams &check_config);
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_FAST_DETECTOR_HPP
