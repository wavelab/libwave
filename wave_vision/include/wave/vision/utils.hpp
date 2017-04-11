#ifndef WAVE_VISION_UTILS_HPP
#define WAVE_VISION_UTILS_HPP

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "wave/utils/utils.hpp"

namespace wave {

void mat2cvmat(MatX A, cv::Mat &B);
void cvmat2mat(cv::Mat A, MatX &B);
void cvpts2mat(std::vector<cv::Point2f> points, MatX &mat);
void cvmatconcat(cv::Mat img1, cv::Mat img2, cv::Mat &out);
void projection_matrix(Mat3 K, Mat3 R, Vec3 t, MatX &P);
void projection_matrix(Mat3 K, Vec3 rpy, Vec3 t, MatX &P);
void normalize_2dpts(double image_width, double image_height, MatX &pts);

}  // end of wave namespace
#endif
