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
 * @param G_p_GC translation from world origin to camera, in world frame
 * @param G_p_GF translation from world origin to feature, in world frame
 * @param result measurement in image frame (pixels)
 * @return true if feature is in front of the camera.
 *
 * @note this function is meant for testing; more sophisticated third-party
 * camera models should normally be used.
 */
template <typename T>
bool pinholeProject(const Eigen::Matrix<T, 3, 3> &K,
                    const Eigen::Matrix<T, 3, 3> &R_GC,
                    const Eigen::Matrix<T, 3, 1> &G_p_GC,
                    const Eigen::Matrix<T, 3, 1> &G_p_GF,
                    Eigen::Matrix<T, 2, 1> &result) {
    // Note R_GC is is the orientation of the camera in the world frame.
    // R_CG is the rotation that transforms *points* in the world frame to the
    // camera frame.
    Eigen::Matrix<T, 3, 3> R_CG = R_GC.transpose();

    // Make extrinsic matrix
    Eigen::Matrix<T, 3, 4> extrinsic{};
    extrinsic.topLeftCorner(3, 3) = R_CG;
    extrinsic.topRightCorner(3, 1) = -R_CG * G_p_GC;
    extrinsic(2, 3) = T(1.0);

    Eigen::Matrix<T, 4, 1> landmark_homogeneous;
    landmark_homogeneous << G_p_GF, T(1);

    // project
    Eigen::Matrix<T, 3, 1> homogeneous = K * extrinsic * landmark_homogeneous;

    // get image coordinates from homogenous coordinates
    result = homogeneous.head(2) / homogeneous(2);

    // check cheirality
    return (homogeneous(2) > T(0));
}

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
