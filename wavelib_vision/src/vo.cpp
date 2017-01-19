#include "slam/vision/vo.hpp"


namespace slam {

VisualOdometry::VisualOdometry(void)
{
    this->configured = false;

    this->focal_length = 0.0;
    this->principle_point = cv::Point2f(0.0, 0.0);
}

int VisualOdometry::configure(Mat3 K)
{
    this->configured = true;

    this->focal_length = K(0, 0);  // fx
    this->principle_point = cv::Point2f(K(0, 2), K(1, 2));  // cx, cy

    return 0;
}

int VisualOdometry::featureTracking(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2,
    std::vector<float> &errors,
    std::vector<uchar> &status
)
{
    int correlation_index;
    cv::Point2f pt;
    cv::Size win_size;
    cv::TermCriteria term_crit;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts_1.size() == 0) {
        return -2;
    }

    // setup
    correlation_index = 0;
    win_size = cv::Size(21, 21);
    term_crit = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        40,
        0.3
    );

    // optical flow
    cv::calcOpticalFlowPyrLK(
        img_1,
        img_2,
        pts_1,
        pts_2,
        status,
        errors,
        win_size,
        3,
        term_crit,
        0,
        0.001
    );

    // // get rid of points for which the KLT tracking failed or those who
    // // have gone outside the frame
    // correlation_index = 0;
    // for (int i = 0; i < (int) status.size(); i++) {
    //     pt = pts_2.at(i - correlation_index);
    //
    //     if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
    //         if ((pt.x < 0) || (pt.y < 0)) {
    //             status.at(i) = 0;
    //         }
    //         pts_1.erase(pts_1.begin() + i - correlation_index);
    //         pts_2.erase(pts_2.begin() + i - correlation_index);
    //         correlation_index++;
    //     }
    // }

    return 0;
}

int VisualOdometry::measure(
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2
)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts_1.size() != pts_2.size()) {
        return -2;
    } else if (pts_1.size() < 10 || pts_2.size() < 10) {
        return -3;
    }

    // essential matrix
    this->E = cv::findEssentialMat(
        pts_1,
        pts_2,
        this->focal_length,
        this->principle_point,
        cv::RANSAC,  // outlier rejection method
        0.999,  // threshold
        1.0  // confidence level
    );
    if (this->E.rows != 3 || this->E.cols != 3) {
        return -4;
    }

    // recover pose
    cv::recoverPose(
        this->E,
        pts_1,
        pts_2,
        this->R,
        this->t,
        this->focal_length,
        this->principle_point
    );

    return 0;
}

int VisualOdometry::drawOpticalFlow(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> pts_1,
    std::vector<cv::Point2f> pts_2,
    cv::Mat &output
)
{
    cv::Point2f p;
    cv::Point2f q;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // draw flow lines
    cvmatconcat(img_1, img_2, output);
    for (int i = 0; i < std::min(pts_1.size(), pts_2.size()); i++) {
        p.x = pts_1[i].x;
        p.y = pts_1[i].y;

        q.x = img_1.size().width + pts_2[i].x;
        q.y = pts_2[i].y;

        cv::arrowedLine(output, p, q, cv::Scalar(0, 0, 255), 1, 8, 0, 0.005);
    }
}

} // end of slam namespace
