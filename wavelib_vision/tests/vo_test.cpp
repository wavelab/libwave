#include <gtest/gtest.h>

#include "slam/vision/utils.hpp"
#include "slam/vision/fast.hpp"
#include "slam/vision/vo.hpp"

#define TEST_IMAGE_1 "tests/data/vo/left.jpg"
#define TEST_IMAGE_2 "tests/data/vo/right.jpg"
#define TEST_FRAME_1 "tests/data/vo/frame_1.jpg"
#define TEST_FRAME_2 "tests/data/vo/frame_2.jpg"


TEST(VisualOdometry, constructor)
{
    slam::VisualOdometry vo;

    ASSERT_EQ(false, vo.configured);
}

TEST(VisualOdometry, configure)
{
    slam::Mat3 K;
    slam::Camera camera;
    slam::VisualOdometry vo;

    K(0, 0) = 1.0;
    K(0, 2) = 2.0;
    K(1, 2) = 3.0;

    vo.configure(K);
    ASSERT_EQ(true, vo.configured);
}

TEST(VisualOdometry, featureTracking)
{
    slam::Mat3 K;
    cv::Mat mask, frame;
    cv::Mat img_1, img_2, img_combined;
    slam::FastDetector fast;
    slam::VisualOdometry vo;
    std::vector<cv::Point2f> cvpts1, cvpts2;
    std::vector<float> errors;
    std::vector<uchar> status;

    // setup
    fast.configure(55, true);
    K(0, 0) = 1.0;
    K(0, 2) = 0.0;
    K(1, 2) = 0.0;
    vo.configure(K);

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);
    fast.detect(img_1, cvpts1);

    // test and assert
    vo.featureTracking(img_1, img_2, cvpts1, cvpts2, errors, status);
    ASSERT_EQ(cvpts2.size(), cvpts1.size());
    ASSERT_EQ(cvpts2.size(), errors.size());
    ASSERT_EQ(cvpts2.size(), status.size());

    // // record tracked points
    // slam::MatX pts1, pts2;
    // slam::convert_cvpts(cvpts1, pts1);
    // slam::convert_cvpts(cvpts2, pts2);
    // slam::mat2csv("/tmp/pts1.dat", pts1);
    // slam::mat2csv("/tmp/pts2.dat", pts2);

    // // display tracked features
    // vo.drawOpticalFlow(img_1, img_2, cvpts1, cvpts2, img_combined);
    // cv::imshow("Test Feature Tracking", img_combined);
    // cv::waitKey(2000);
}

// TEST(VisualOdometry, measure)
// {
//     cv::Mat K;
//     cv::Mat mask;
//     cv::Mat frame;
//     cv::Mat img_1;
//     cv::Mat img_2;
//     slam::FastDetector fast;
//     slam::VisualOdometry vo;
//     std::vector<cv::Point2f> pts_1;
//     std::vector<cv::Point2f> pts_2;
//     std::vector<float> errors;
//     std::vector<uchar> status;
//
//     // // setup
//     // fast.configure(60, true);
//     K.at<double>(0, 0) = 1.0;  // fx
//     K.at<double>(1, 1) = 1.0;  // fy
//     K.at<double>(0, 2) = 0.0;  // cx
//     K.at<double>(1, 0) = 0.0;  // cy
//     K.at<double>(2, 2) = 1.0;  // 1
//     vo.configure(K);
//     vo.focal_length = 1.0;
//     vo.principle_point = cv::Point2f(0.0, 0.0);
//     // img_1 = cv::imread(TEST_FRAME_1);
//     // img_2 = cv::imread(TEST_FRAME_1);
//     // fast.detect(img_1, pts_1);
//     //
//     //
//     // // test and assert
//     // std::cout << pts_1.size() << std::endl;
//     // vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
//     // ASSERT_EQ(pts_1.size() == pts_2.size());
//     // ASSERT_EQ(errors.size() == pts_2.size());
//     // ASSERT_EQ(status.size() == pts_2.size());
//     //
//     // vo.displayOpticalFlow(img_2, pts_1, pts_2);
//     // cv::imshow("Frame 1", img_1);
//     // cv::imshow("Frame 2", img_2);
//     // cv::waitKey(0);
//
//     // std::cout << pts_1.size() << std::endl;
//     // std::cout << pts_2.size() << std::endl;
//     // std::cout << errors.size() << std::endl;
//     // std::cout << status.size() << std::endl;
//
//     // for (int i = 0; i < pts_1.size(); i++) {
//     //     std::cout << pts_1[i] << "\t";
//     //     std::cout << pts_2[i] << "\n";
//     // }
//
//     // for (int i = 0; i < 40; i++) {
//     //     pts_1.push_back(cv::Point2f(5.0, (float) i));
//     //     pts_2.push_back(cv::Point2f(5.0, (float) i + 10));
//     // }
//
//
//     cv::Mat dis_coef;
//
//     std::cout << pts_1 << std::endl;
//
//     cv::undistortPoints(pts_1, pts_1, K, dis_coef);
//     cv::undistortPoints(pts_2, pts_2, K, dis_coef);
//
//     std::cout << pts_1 << std::endl;
//
//     // cv::Mat E = cv::findEssentialMat(
//     //     pts_1,
//     //     pts_2,
//     //     1.0,
//     //     cv::Point2f(0.0, 0.0),
//     //     cv::RANSAC,  // outlier rejection method
//     //     0.999,       // threshold
//     //     1.0          // confidence level
//     // );
//     // std::cout << E << std::endl;
//
//     // cv::Mat Fun = cv::findFundamentalMat(
//     //     pts_1,
//     //     pts_2,
//     //     cv::FM_8POINT
//     // );
//     //
//     // Eigen::Matrix3d K;
//     // K << 1.0, 0.0, 0.0,
//     //      0.0, 1.0, 0.0,
//     //      0.0, 0.0, 1.0;
//     //
//     // Eigen::Matrix3d F;
//     // F << Fun.at<double>(0, 0), Fun.at<double>(0, 1), Fun.at<double>(0, 2),
//     //      Fun.at<double>(1, 0), Fun.at<double>(1, 1), Fun.at<double>(1, 2),
//     //      Fun.at<double>(2, 0), Fun.at<double>(2, 1), Fun.at<double>(2, 2);
//     // Eigen::Matrix3d result = (K.transpose() * F * K);
//     //
//     // std::cout << std::endl;
//     // std::cout << result << std::endl;
//
//     // // draw flow lines
//     // cv::Mat image = cv::Mat(300, 300, CV_8UC3, double(0));
//     // cv::Point2f p;
//     // cv::Point2f q;
//     //
//     // for (int i = 0; i < std::min(pts_1.size(), pts_2.size()); i++) {
//     //     std::cout << i << std::endl;
//     //     p.x = pts_1[i].x;
//     //     p.y = pts_1[i].y;
//     //
//     //     q.x = pts_2[i].x;
//     //     q.y = pts_2[i].y;
//     //
//     //     cv::arrowedLine(image, p, q, cv::Scalar(0, 0, 255), 1);
//     // }
//     //
//     // cv::imshow("image", image);
//     // cv::waitKey(0);
//
//     // cv::Mat R;
//     // cv::Mat t;
//     //
//     // int retval = cv::recoverPose(
//     //     E,
//     //     pts_1,
//     //     pts_2,
//     //     R,
//     //     t,
//     //     1.0,
//     //     cv::Point2f(0.0, 0.0)
//     // );
//     //
//     // std::cout << retval << std::endl;
//     // std::cout << t << std::endl;
//
//     // int retval;
//     // retval = vo.measure(pts_1, pts_2);
//     // std::cout << retval << std::endl;
//     // std::cout << vo.t << std::endl;
//     // Eigen::Vector3d vec;
//     // vec << vo.t.at<double>(0), vo.t.at<double>(1), vo.t.at<double>(2);
//     // std::cout << vec.norm() << std::endl;
//
//     // vo.displayOpticalFlow(img_2, pts_1, pts_2);
//     // cv::imshow("Test Feature Tracking", img_2);
//     // cv::waitKey(0);
// }

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
