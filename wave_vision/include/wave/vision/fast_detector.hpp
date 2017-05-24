#ifndef WAVE_FAST_DETECTOR_HPP
#define WAVE_FAST_DETECTOR_HPP

// Libwave Headers
#include "wave/vision/feature_detector.hpp"

namespace wave {

struct FASTParams {
    int threshold;
    bool nonmax_suppression;
    int type;
};

class FASTDetector : public FeatureDetector<cv::Mat> {
 public:
    // Public Functions
    // -------------------------------------------------------------------------
    FASTDetector();
    FASTDetector(FASTParams& config);
    FASTDetector(const std::string& config_path);
    ~FASTDetector();
    void configure(FASTParams& new_config);
    FASTParams getConfiguration();
    std::vector<cv::KeyPoint>& detectFeatures(const cv::Mat& image);

 private:
    // OpenCV Variables
    // -------------------------------------------------------------------------
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

    // Private Functions
    // -------------------------------------------------------------------------
    void checkConfiguration(FASTParams& check_config);
};

} // end of namespace wave



#endif //WAVE_FAST_DETECTOR_HPP
