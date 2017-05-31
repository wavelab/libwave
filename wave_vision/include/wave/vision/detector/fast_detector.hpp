/**
 * @file
 * FAST Feature Detector implementation, derived from Feature Detector base
 * class.
 * @ingroup vision
 */
#ifndef WAVE_FAST_DETECTOR_HPP
#define WAVE_FAST_DETECTOR_HPP

/** C++ Headers */
#include <string>
#include <vector>

/** Libwave Headers */
#include "wave/vision/detector/feature_detector.hpp"

/** The wave namespace */
namespace wave {
/** @addtogroup vision
 *  @{ */

/**
 *  FASTParams struct contains the configuration parameters for the
 *  FastFeatureDetector.
 *
 */
struct FASTParams {
    /**
     *  Threshold on difference between intensity of the central pixel, and
     *  pixels in a circle (Bresenham radius 3) around this pixel.
     *  10 (recommended), must be greater than zero.
     */
    int threshold;

    /**
     *  Removes keypoints in adjacent locations
     *  true (recommended), or false
     */
    bool nonmax_suppression;

    /**
     *  Neighbourhood, as defined in the paper by Rosten. May only be the
     *  following:
     *  0 = TYPE_5_8
     *  1 = TYPE_7_12
     *  2 = TYPE_9_16 (recommended)
     *
     *  The neighbourhood refers to the pixel circumference and number of
     *  pixels that must be brighter or darker than the center pixel for the
     *  algorithm to deem the point as a corner. For example, TYPE_9_16
     *  requires 9 consecutive pixels out of a 16 pixel circumference circle to
     *  be brighter or darker than the center pixel.
     */
    int type;
};

/**
 * Representation of a feature detector using the FAST algorithm.
 *
 * Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 * Further reference on the FastFeatureDetector can be found
 * [here][opencv_feature_detectors].
 *
 * [opencv_feature_detectors]:
 * http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public FeatureDetector {
 public:
    /** Default Constructor */
    FASTDetector();

    /** Constructs a FastFeatureDetector using parameters specified by the user
     *
     *  @param config, containing the desired threshold, nonmax_suppression,
     *  and type values.
     *
     */
    explicit FASTDetector(const FASTParams &config);

    /** Constructs a FastFeatureDetector using parameters extracted from the
     *  fast.yaml configuration file
     *
     *  @param config_path, the path to the location of the fast.yaml config
     *  file
     */
    explicit FASTDetector(const std::string &config_path);

    /** Destructor */
    ~FASTDetector();

    /** Reconfigures the FastFeatureDetector object with new values requested by
     *  the user
     *
     * @param new_config, containing the desired configuration values.
     */
    void configure(const FASTParams &new_config);

    /** Returns the current configuration parameters being used by the
     *  FastFeatureDetector.
     *
     * @return a struct containing the current configuration values.
     */
    FASTParams getConfiguration();

    /** Detects features in an image using the FastFeatureDetector.
     *
     *  @param image, the image to detect features in.
     *  @return a vector containing all of the keypoints found within the image.
     */
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image);

 private:
    /** The pointer to the wrapped cv::FastFeatureDetector object */
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

    /** Checks whether the desired configuration is valid.
     *
     *  The threshold value must be greater than zero, while the type must be
     *  0, 1, or 2.
     *
     *  @param check_config, containing the desired configuration values.
     */
    void checkConfiguration(const FASTParams &check_config);
};

/** @} end of group */
}  // end of namespace wave

#endif  // WAVE_FAST_DETECTOR_HPP
