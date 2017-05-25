/**
 * @file
 * Common computer vision utilities
 * @ingroup vision
 */
#ifndef WAVE_VISION_COMMON_HPP
#define WAVE_VISION_COMMON_HPP

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "wave/utils/utils.hpp"

namespace wave {

/** Calculate theoretical focal length based on field of view (fov) in radians
 * and image width in pixels */
double focal_length(double fov, double image_width);

/** Calculate theoretical focal length based on field of view (fov) in
 * horizontal and vertical plane in radians and image width and height in
 * pixels */
Vec2 focal_length(double hfov,
                  double vfov,
                  double image_width,
                  double image_height);

/** Combine 2 matrices together to form a third, this function was design to
 * concatenate two image matrices `img1` and `img2` so they can form a third
 * side by side image `out` */
void matconcat(const cv::Mat &img1, const cv::Mat &img2, cv::Mat &out);

/** Create projection matrix `P` based on camera intrinsics `K`, camera
 * rotation `R` and camera position `t` */
void projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t, MatX &P);

}  // end of wave namespace
#endif
