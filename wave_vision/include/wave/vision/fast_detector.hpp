#ifndef WAVE_FAST_FEATURE_DETECTOR_HPP
#define WAVE_FAST_FEATURE_DETECTOR_HPP

#include "wave/vision/feature_detector.hpp"

namespace wave {

class FASTDetector : public FeatureDetector <Image> {
 public:
    explicit FASTDetector(const std::string&);
    ~FASTDetector();
    void configureDetector(int new_threshold,
                           bool new_nonmax_suppression,
                           int new_type);
    void loadImage(Image& source_image);
    // const std::vector<Keypoint>& detectFeatures();


 private:
    // Member Variables
    // -------------------------------------------------------------------------

    // Configuration Parameters
    int threshold;
    bool nonmax_suppression;
    int type;

    // OpenCV Variables
    // -------------------------------------------------------------------------
    static cv::Ptr<cv::FastFeatureDetector> fast_detector;
    cv::Ptr<Image> image;
    cv::Ptr<Keypoint> keypoints;

    // Private Functions
    // -------------------------------------------------------------------------
    void checkConfiguration(int new_threshold, int new_type);
};

} // end of namespace wave



#endif //WAVE_FAST_FEATURE_DETECTOR_HPP
