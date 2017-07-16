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

/** Measure a 3D point using a simple pinhole camera model.
 *
 * @param K camera intrinsic matrix
 * @param R_GC orientation of camera in world frame
 * @param G_p_C_G translation from world origin to camera, in world frame
 * @param G_p_F_G translation from world origin to feature, in world frame
 * @param result measurement in image frame (pixels)
 * @return true if feature is in front of the camera.
 *
 * @note this function is meant for testing; more sophisticated third-party
 * camera models should normally be used.
 */
bool pinholeProject(const Mat3 &K,
                    const Mat3 &R_GC,
                    const Vec3 &G_p_C_G,
                    const Vec3 &G_p_F_G,
                    Vec2 &result);

/** Convert a single cv::KeyPoint to Vec2
 *
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
Vec2 convertKeypoint(const cv::KeyPoint &keypoint);

/** Convert a single cv::Point2f to Vec2
 *
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
Vec2 convertKeypoint(const cv::Point2f &keypoint);

/** Convert a single Vec2 to cv::Point2f
 *
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
cv::Point2f convertKeypoint(const Vec2 &keypoint);

/** Convert a vector of cv::KeyPoint to a vector of Vec2
 *
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<Vec2> convertKeypoints(const std::vector<cv::KeyPoint> &keypoints);

/** Convert a vector of cv::Point2f to a vector of Vec2
 *
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<Vec2> convertKeypoints(const std::vector<cv::Point2f> &keypoints);

/** Convert a vector of Vec2 to a vector of cv::Point2f
 *
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<cv::Point2f> convertKeypoints(const std::vector<Vec2> &keypoints);

/** Reads images from file into a vector
 *
 * @param images_path location of images
 * @return all read images
 */
std::vector<cv::Mat> readImageSequence(const std::string &images_path);

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_COMMON_HPP
