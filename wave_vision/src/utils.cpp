#include "wave/vision/utils.hpp"

namespace wave {

double focal_length(double fov, double image_width) {
    return (image_width / 2.0) / tan(deg2rad(fov) / 2.0);
}

Vec2 focal_length(double hfov,
                  double vfov,
                  double image_width,
                  double image_height) {
    Vec2 focal;

    focal(0) = focal_length(hfov, image_width);
    focal(1) = focal_length(vfov, image_height);

    return focal;
}

void projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t, MatX &P) {
    MatX extrinsics(3, 4);
    extrinsics.block(0, 0, 3, 3) = R;
    extrinsics.block(0, 3, 3, 1) = t;

    P.resize(3, 4);
    P = K * extrinsics;
}

void convertKeypoint(const cv::KeyPoint &keypoint, Vec2 &vec_keypoint) {
    vec_keypoint(0) = keypoint.pt.x;
    vec_keypoint(1) = keypoint.pt.y;
}

void convertKeypoints(const std::vector<cv::KeyPoint> &keypoints,
                      std::vector<Vec2> &vec_keypoints) {
    for (const auto &k : keypoints) {
        vec_keypoints.emplace_back(k.pt.x, k.pt.y);
    }
}
}  // namespace wave
