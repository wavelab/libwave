#ifndef __wavelib_VISION_GOOD_HPP__
#define __wavelib_VISION_GOOD_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace wavelib {

class GoodDetector
{
public:
    bool configured;

    int max_corners;
    double quality_level;
    double min_dist;
    int block_size;
    bool use_harris;
    double k;

    cv::Mat mask;

    GoodDetector(void);
    int configure(void);
    int detect(cv::Mat &image, std::vector<cv::Point2f> &points);
};

}  // end of wavelib namespace
#endif
