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
/** @addtogroup vision
 *  @{ */

/** Calculate theoretical focal length
 *
 * @param fov field of view in radians
 * @param image_width image width in pixels
 */
double focal_length(double fov, double image_width);

/** Calculate theoretical focal length in two dimensions
 * @param hfov, vfov horizontal and vertical field of view in radians
 * @param image_width, image_height image dimensions in pixels
 */
Vec2 focal_length(double hfov,
                  double vfov,
                  double image_width,
                  double image_height);

/** Combine 2 matrices together to form a third
 *
 * This function was design to concatenate two image matrices `img1` and `img2`
 * so they can form a third side by side image, `out`
 */
void matconcat(const cv::Mat &img1, const cv::Mat &img2, cv::Mat &out);

/** Calculate camera projection matrix
 *
 * @param K camera intrinsic matrix
 * @param R camera rotation matrix
 * @param t camera position
 * @param[out] P camera projection matrix
 */
void projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t, MatX &P);

/** @} end of group */
}  // end of wave namespace
#endif
