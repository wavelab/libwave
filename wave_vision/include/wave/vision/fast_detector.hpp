/**
 * @file
 * Fast Feature Detector implementation, derived from Feature Detector base
 * class.
 * @ingroup vision
 */
#ifndef WAVE_FAST_DETECTOR_HPP
#define WAVE_FAST_DETECTOR_HPP

/** Libwave Headers */
#include "wave/vision/feature_detector.hpp"

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
    int threshold;
    bool nonmax_suppression;
    int type;
};

/**
 * Representation of a feature detector using the FAST algorithm.
 *
 * Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 * Further reference on the FastFeatureDetector can be found
 * [here][opencv_feature_detectors].
 *
 * [opencv_feature_detectors]: http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public FeatureDetector {
 public:
    /** Default Constructor */
    FASTDetector();

    /** Constructs a FastFeatureDetector using parameters specified by the user
     *
     *  @param config, a FASTParams struct containing the desired threshold,
     *  nonmax_suppression, and type values.
     *
     */
    FASTDetector(FASTParams& config);

    /** Constructs a FastFeatureDetector using parameters extracted from the
     *  fast.yaml configuration file
     *
     *  @param config_path, the path to the location of the fast.yaml config
     *  file
     */
    FASTDetector(const std::string& config_path);

    /** Destructor */
    ~FASTDetector();

    /** Reconfigures the FastFeatureDetector object with new values requested by
     *  the user
     *
     * @param new_config, a FASTParams struct containing the desired
     * configuration values.
     */
    void configure(FASTParams& new_config);

    /** Returns the current configuration parameters being used by the
     *  FastFeatureDetector.
     *
     * @return FASTParams, a struct containing the current configuration values.
     */
    FASTParams getConfiguration();

    /** Detects features in an image using the FastFeatureDetector.
     *
     *  @param image, a cv::Mat object of the image to detect features in.
     *  @return std::vector<cv::KeyPoint>, a vector containing all of the
     *  keypoints found within the image.
     */
    std::vector<cv::KeyPoint>& detectFeatures(const cv::Mat& image);

 private:
    /** The smart pointer to the wrapped cv::FastFeatureDetector object */
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

   /** Checks whether the desired configuration is valid.
    *
    *  The threshold value must be greater than zero, while the type must be
    *  0, 1, or 2.
    *
    *  @param check_config, the FASTParams struct containing the desired
    *  configuration values.
    */
    void checkConfiguration(FASTParams& check_config);
};

} // end of namespace wave

#endif //WAVE_FAST_DETECTOR_HPP
