#include "wave/vision/common.hpp"

namespace wave {

double focal_length(double hfov, double image_width) {
    return (image_width / 2.0) / tan(deg2rad(hfov) / 2.0);
}

Vec2 focal_length(double hfov,
                  double vfov,
                  double image_width,
                  double image_height) {
    Vec2 focal;

    focal(0) = (image_width / 2.0) / tan(deg2rad(hfov) / 2.0);
    focal(1) = (image_height / 2.0) / tan(deg2rad(vfov) / 2.0);

    return focal;
}

void matconcat(const cv::Mat &img1, const cv::Mat &img2, cv::Mat &out) {
    // setup
    cv::Size size1 = img1.size();
    cv::Size size2 = img2.size();
    out = cv::Mat(size1.height, size1.width + size2.width, img1.type());

    // copy image 1 to the left
    out.adjustROI(0, 0, 0, -size2.width);
    img1.copyTo(out);

    // copy image 2 to the right
    out.adjustROI(0, 0, -size1.width, size2.width);
    img2.copyTo(out);

    // restore original roi
    out.adjustROI(0, 0, size1.width, 0);
}

void projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t, MatX &P) {
    MatX extrinsics(3, 4);
    extrinsics.block(0, 0, 3, 3) = R;
    extrinsics.block(0, 3, 3, 1) = t;

    P.resize(3, 4);
    P = K * extrinsics;
}

}  // end of wave namespace
