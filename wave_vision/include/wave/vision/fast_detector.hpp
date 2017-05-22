#ifndef WAVE_FAST_DETECTOR_HPP
#define WAVE_FAST_DETECTOR_HPP

#include "wave/vision/feature_detector.hpp"

namespace wave {

class FASTDetector : public FeatureDetector<cv::Mat> {
 public:
    FASTDetector(const std::string& config_path);
    ~FASTDetector();
    void configure(int  new_threshold,
                   bool new_nonmax_suppression,
                   int  new_type);
    void getConfiguration(int&  current_threshold,
                          bool& current_nonmax_suppression,
                          int&  current_type);
    std::vector<cv::KeyPoint>& detectFeatures(const cv::Mat& image);

 private:
    // Member Variables
    // -------------------------------------------------------------------------

    // FASTDetector-specific Configuration Parameters
    int threshold;
    bool nonmax_suppression;
    int type;

    // OpenCV Variables
    // -------------------------------------------------------------------------
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

    // Private Functions
    // -------------------------------------------------------------------------
    void checkConfiguration(int new_threshold, int new_type);
};

} // end of namespace wave



#endif //WAVE_FAST_DETECTOR_HPP
