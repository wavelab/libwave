#ifndef __wave_VISION_VO_HPP__
#define __wave_VISION_VO_HPP__

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/camera.hpp"


namespace wave {

class VisualOdometry
{
public:
    bool configured;

    double focal_length;
    cv::Point2f principle_point;

    cv::Mat mask;
    cv::Mat E;
    cv::Mat R;
    cv::Mat t;

    VisualOdometry(void);
    int configure(Mat3 K);
    int featureTracking(
        cv::Mat img_1,
        cv::Mat img_2,
        std::vector<cv::Point2f> &pts_1,
        std::vector<cv::Point2f> &pts_2,
        std::vector<float> &errors,
        std::vector<uchar> &status
    );
    int measure(
        std::vector<cv::Point2f> &pts_1,
        std::vector<cv::Point2f> &pts_2
    );
    int drawOpticalFlow(
        cv::Mat img_1,
        cv::Mat img_2,
        std::vector<cv::Point2f> pts_1,
        std::vector<cv::Point2f> pts_2,
        cv::Mat &output
    );
};

} // end of wave namespace
#endif
