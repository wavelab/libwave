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

bool pinholeProject(const Mat3 &K,
                    const Mat3 &R_GC,
                    const Vec3 &G_p_C_G,
                    const Vec3 &G_p_F_G,
                    Vec2 &result) {
    // Note R_GC is is the orientation of the camera in the world frame.
    // R_CG is the rotation that transforms *points* in the world frame to the
    // camera frame.
    Mat3 R_CG = R_GC.transpose();

    // Make extrinsic matrix
    MatX extrinsic(3, 4);
    extrinsic.topLeftCorner<3, 3>() = R_CG;
    extrinsic.topRightCorner<3, 1>() = -R_CG * G_p_C_G;
    extrinsic(2, 3) = 1;

    Vec4 landmark_homogeneous;
    landmark_homogeneous << G_p_F_G, 1.;

    // project
    Vec3 homogeneous = K * extrinsic * landmark_homogeneous;

    // get image coordinates from homogenous coordinates
    result = homogeneous.head<2>() / homogeneous(2);

    // check cheirality
    return (homogeneous(2) > 0);
}

Vec2 convertKeypoint(const cv::KeyPoint &keypoint) {
    Vec2 vec_keypoint(keypoint.pt.x, keypoint.pt.y);

    return vec_keypoint;
}

Vec2 convertKeypoint(const cv::Point2f &keypoint) {
    Vec2 vec_keypoint(keypoint.x, keypoint.y);

    return vec_keypoint;
}

cv::Point2f convertKeypoint(const Vec2 &keypoint) {
    cv::Point2f cv_keypoint((float) keypoint(0), (float) keypoint(1));

    return cv_keypoint;
}

std::vector<Vec2> convertKeypoints(const std::vector<cv::KeyPoint> &keypoints) {
    std::vector<Vec2> vec_keypoints;

    for (const auto &k : keypoints) {
        vec_keypoints.emplace_back(k.pt.x, k.pt.y);
    }

    return vec_keypoints;
}

std::vector<Vec2> convertKeypoints(const std::vector<cv::Point2f> &keypoints) {
    std::vector<Vec2> vec_keypoints;

    for (const auto &k : keypoints) {
        vec_keypoints.emplace_back(k.x, k.y);
    }

    return vec_keypoints;
}

std::vector<cv::Point2f> convertKeypoints(const std::vector<Vec2> &keypoints) {
    std::vector<cv::Point2f> cv_keypoints;

    for (const auto &k : keypoints) {
        cv_keypoints.emplace_back((float) k(0), (float) k(1));
    }

    return cv_keypoints;
}

std::vector<cv::Mat> readImageSequence(const std::string &path) {
    cv::String img_path(path);
    std::vector<cv::String> files;
    std::vector<cv::Mat> image_sequence;
    int num_images;

    cv::VideoCapture sequence(img_path, cv::CAP_IMAGES);

    num_images = (int) sequence.get(cv::CAP_PROP_FRAME_COUNT);

    if (sequence.isOpened()) {
        for (double i = 0; i < num_images; ++i) {
            cv::Mat image;
            sequence >> image;
            image_sequence.push_back(image);
        }
    } else {
        throw std::length_error("No images in image sequence!");
    }

    return image_sequence;
}

}  // namespace wave

namespace YAML {
bool convert<cv::Mat>::decode(const Node &node, cv::Mat &out) {
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    const auto &data = node["data"];

    // Check `rows` and `cols` values
    if (rows <= 0 || cols <= 0) {
        return false;
    }

    // Check data node is a list of the right length
    std::size_t expected_size = rows * cols;
    if (!data.IsSequence() || data.size() != expected_size) {
        return false;
    }

    // Copy it to destination
    out = cv::Mat{rows, cols, CV_64F};
    for (int i = 0, index = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            out.at<double>(i, j) = node["data"][index++].as<double>();
        }
    }
    return true;
}
}  //  namespace YAML
