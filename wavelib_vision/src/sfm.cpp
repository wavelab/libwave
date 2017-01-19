#include "slam/vision/sfm.hpp"


namespace slam {

SFMPose::SFMPose(void)
{

}

SFM::SFM(void)
{
    this->configured = false;
}

int SFM::configure(Mat3 K)
{
    this->configured = true;

    this->K = K;
}

int SFM::recoverPose(MatX pts1, MatX pts2, SFMPose &pose)
{
    cv::Mat E, R, t;
    double focal_length;
    cv::Point2f principle_point;
    std::vector<cv::Point2f> cvpts1;
    std::vector<cv::Point2f> cvpts2;

    // setup
    focal_length = this->K(0, 0);
    principle_point = cv::Point2f(this->K(0, 2), this->K(1, 2));

    // convert matrix to cv 2d points
    for (int i = 0; i < pts1.rows(); i++) {
        cvpts1.push_back(cv::Point2f(pts1(i, 0), pts1(i, 1)));
        cvpts2.push_back(cv::Point2f(pts2(i, 0), pts2(i, 1)));
    }

    // essential matrix
    E = cv::findEssentialMat(
        cvpts1,
        cvpts2,
        focal_length,
        principle_point,
        cv::RANSAC,  // outlier rejection method
        0.999,  // confidence level
        1  // threshold (pixels)
    );
    if (E.rows != 3 || E.cols != 3) {
        return -1;
    }

    // recover pose
    cv::recoverPose(
        E,
        cvpts1,
        cvpts2,
        R,
        t,
        focal_length,
        principle_point
    );

    // set pose
    pose.R(0, 0) = R.at<double>(0, 0);
    pose.R(0, 1) = R.at<double>(0, 1);
    pose.R(0, 2) = R.at<double>(0, 2);
    pose.R(1, 0) = R.at<double>(1, 0);
    pose.R(1, 1) = R.at<double>(1, 1);
    pose.R(1, 2) = R.at<double>(1, 2);
    pose.R(2, 0) = R.at<double>(2, 0);
    pose.R(2, 1) = R.at<double>(2, 1);
    pose.R(2, 2) = R.at<double>(2, 2);

    pose.t(0) = t.at<double>(0, 0);
    pose.t(1) = t.at<double>(1, 0);
    pose.t(2) = t.at<double>(2, 0);

    return 0;
}

} // end of slam namespace
