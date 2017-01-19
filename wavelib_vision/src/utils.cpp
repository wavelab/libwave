#include "slam/vision/utils.hpp"


namespace slam {

void mat2cvmat(MatX A, cv::Mat &B)
{
    B = cv::Mat(A.rows(), A.cols(), CV_8UC1);
    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            B.at<double>(i, j) = A(i, j);
        }
    }
}

void cvmat2mat(cv::Mat A, MatX &B)
{
    B.resize(A.rows, A.cols);
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.cols; j++) {
            B(i, j) = A.at<double>(i, j);
        }
    }
}

void cvpts2mat(std::vector<cv::Point2f> points, MatX &mat)
{
    cv::Point2f p;

    mat.resize(points.size(), 3);
    for (int i = 0; i < points.size(); i++) {
        p = points[i];
        mat(i, 0) = p.x;
        mat(i, 1) = p.y;
        mat(i, 2) = 1.0;
    }
}

void cvmatconcat(cv::Mat img1, cv::Mat img2, cv::Mat &out)
{
    cv::Size size1;
    cv::Size size2;

    // setup
    size1 = img1.size();
    size2 = img2.size();
    out = cv::Mat(
        size1.height,
        size1.width + size2.width,
        img1.type()
    );

    // copy image 1 to the left
    out.adjustROI(0, 0, 0, -size2.width);
    img1.copyTo(out);

    // copy image 2 to the right
    out.adjustROI(0, 0, -size1.width, size2.width);
    img2.copyTo(out);

    // restore original roi
    out.adjustROI(0, 0, size1.width, 0);
}

void projection_matrix(Mat3 K, Mat3 R, Vec3 t, MatX &P)
{
    P.resize(3, 4);
    P.block(0, 0, 3, 3) = R;
    P.block(0, 3, 3, 1) = -(R * t);
    P = K * P;
}

void normalize_2dpts(double image_width, double image_height, MatX &pts)
{
    Mat3 N;
    MatX pts_h;

    // convert points to homogeneous coordinates
    pts_h.resize(pts.rows(), 3);
    for (int i = 0; i < pts.rows(); i++) {
        pts_h(i, 0) = pts(i, 0);
        pts_h(i, 1) = pts(i, 1);
        pts_h(i, 2) = 1.0;
    }

    // create normalization matrix N
    N << 2.0 / image_width, 0.0, -1.0,
         0.0, 2.0 / image_width, -1.0,
         0.0, 0.0, 1.0;

    // normalize image points to camera center
    pts_h = (N * pts_h.transpose()).transpose();

    // convert back to 2d coordinates
    for (int i = 0; i < pts.rows(); i++) {
        pts(i, 0) = pts_h(i, 0);
        pts(i, 1) = pts_h(i, 1);
    }
}

} // end of slam namespace
