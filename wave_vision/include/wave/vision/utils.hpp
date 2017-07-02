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

/** Calculate camera projection matrix
 *
 * @param K camera intrinsic matrix
 * @param R camera rotation matrix
 * @param t camera position
 * @param[out] P camera projection matrix
 */
void projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t, MatX &P);

/** Convert a single cv::KeyPoint to a Vec2 object
 *
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
void convertKeypoints(const cv::KeyPoint &keypoint, Vec2 &vec_keypoints);

/** Convert a vector of keypoints to a vector of Vec2
 *
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
void convertKeypoints(const std::vector<cv::KeyPoint> &keypoints,
                      std::vector<Vec2> &vec_keypoints);

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_COMMON_HPP
