#ifndef __SLAM_VISION_CALIBRATION_HPP__
#define __SLAM_VISION_CALIBRATION_HPP__

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include "slam/utils/utils.hpp"
#include "slam/utils/logging.hpp"
#include "slam/vision/chessboard.hpp"


namespace slam {

// ERROR MESSAGES
#define ECALIBDIRPERM "permission denied! while creating [%s]!"
#define ECALIBNOTDIR "[%s] is not a dir!"
#define ECALIBDIREXIST "[%s] already exists!"
#define ECALIBDIR "failed to create [%s]!"

// CALIBRATION STATES
#define IDEL 0
#define CAPTURING 1
#define READY_TO_CALIBRATE 2

// MACROS
#define CALIBRATION_FILENAME "calibration.yaml"


class Calibration
{
public:
    bool configured;
    int state;

    Chessboard chessboard;
    int nb_samples;
    int nb_max_samples;
    std::string save_path;
    YAML::Emitter yaml_config;

    cv::Size image_size;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;
    double reprojection_error;

    Calibration(void);
    int configure(
        std::string calibration_path,
        Chessboard &chessboard,
        cv::Size image_size,
        int nb_max_samples
    );
    bool findChessboardCorners(
        cv::Mat &image,
        std::vector<cv::Point2f> &corners
    );
    int saveImage(cv::Mat &image, std::vector<cv::Point2f> corners);
    int calibrate(
        std::vector<std::vector<cv::Point2f>> image_points,
        cv::Size image_size
    );
    int saveCalibrationOutputs(void);
};

} // end of slam namespace
#endif
