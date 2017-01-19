#ifndef __SLAM_VISION_ORB_HPP__
#define __SLAM_VISION_ORB_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace slam {

class ORB
{
public:
    bool configured;

    int nb_features;
    float scale_factor;
    int nb_levels;
    int edge_threshold;
    int first_level;
    int wta_k;
    int score_type;
    int patch_size;

    cv::ORB detector;

    ORB(void);
    int configure(void);
    int detect(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
    int compute(
        cv::Mat &image,
        std::vector<cv::KeyPoint> &keypoints,
        cv::Mat &descriptors
    );
};

}  // end of slam namespace
#endif
