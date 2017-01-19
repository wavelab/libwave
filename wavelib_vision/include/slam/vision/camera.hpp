#ifndef __SLAM_VISION_CAMERA_HPP__
#define __SLAM_VISION_CAMERA_HPP__

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <yaml-cpp/yaml.h>

#include "slam/utils/logging.hpp"


namespace slam {

class CameraConfig
{
public:
    int camera_index;
    int image_width;
    int image_height;

    float exposure;
    float gain;

    cv::Mat camera_mat;
    cv::Mat rectification_mat;
    cv::Mat distortion_coef;
    cv::Mat projection_mat;
};


class Camera
{
public:
    bool configured;
    cv::VideoCapture *capture;

    int capture_index;
    int image_width;
    int image_height;
    cv::Mat camera_mat;
    cv::Mat distortion_coef;

    Camera(void);
    ~Camera(void);
    int configure(int capture_index, int image_width, int image_height);
    int configure(int capture_index, std::string calibration_file);
    int getFrame(cv::Mat &frame);
    int getUndistortFrame(cv::Mat &frame);
    int saveFrame(cv::Mat &frame, std::string save_path);
    void close(void);
};

} // end of slam namespace
#endif
