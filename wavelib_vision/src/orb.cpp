#include "slam/vision/orb.hpp"


namespace slam {

// ORB DETECTOR
ORB::ORB(void)
{
    this->configured = false;

    this->nb_features = 500;
    this->scale_factor = 1.2f;
    this->nb_levels = 8;
    this->edge_threshold = 31;
    this->first_level = 0;
    this->wta_k = 2;
    this->score_type = cv::ORB::HARRIS_SCORE;
    this->patch_size = 31;
}

int ORB::configure(void)
{
    this->configured = true;

    this->detector = cv::ORB(
        this->nb_features,
        this->scale_factor,
        this->nb_levels,
        this->edge_threshold,
        this->first_level,
        this->wta_k,
        this->score_type,
        this->patch_size
    );

    return 0;
}

int ORB::detect(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints)
{
    this->detector.detect(image, keypoints);
    return 0;
}

int ORB::compute(
    cv::Mat &image,
    std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors
)
{
    this->detector.compute(image, keypoints, descriptors);
    return 0;
}

} // end of slam namespace
