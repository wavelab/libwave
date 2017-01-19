#include "slam/vision/good.hpp"


namespace slam {

GoodDetector::GoodDetector(void)
{
    this->configured = false;
    this->max_corners = 1000;
    this->quality_level = 0.1;
    this->min_dist = 10;
    this->block_size = 3;
    this->use_harris = false;
    this->k = 0.04;
}

int GoodDetector::configure(void)
{
    this->configured = true;
}

int GoodDetector::detect(cv::Mat &image, std::vector<cv::Point2f> &points)
{
    cv::goodFeaturesToTrack(
        image,
        points,
        this->max_corners,
        this->quality_level,
        this->min_dist,
        this->mask,
        this->block_size
    );

    return 0;
}

}  // end of slam namespace
