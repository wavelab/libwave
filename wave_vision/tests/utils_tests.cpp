#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"

namespace wave {

const std::string IMG_FOLDER_PATH =
  "tests/data/tracker_test_sequence/frame0057.jpg";

TEST(VisionCommon, focal_length) {
    double f = focal_length(60, 640);
    EXPECT_FLOAT_EQ(554.25629, f);

    Vec2 f2 = focal_length(60, 60, 640, 640);
    EXPECT_FLOAT_EQ(554.25629, f2(0));
    EXPECT_FLOAT_EQ(554.25629, f2(1));
}

TEST(VisionCommon, projection_matrix) {
    // setup
    double fx = 554.38;
    double fy = 554.38;
    double cx = 320.0;
    double cy = 320.0;

    MatX expected(3, 4);
    expected << fx, 0.0, cx, 0.0,  //
      0.0, fy, cy, 0.0,            //
      0.0, 0.0, 1.0, 0.0;          //

    Mat3 K;
    K << fx, 0.0, cx,  //
      0.0, fy, cy,     //
      0.0, 0.0, 1.0;   //

    Vec3 rpy;
    rpy << deg2rad(0.0), deg2rad(0.0), deg2rad(0.0);

    Mat3 R;
    euler2rot(rpy, 123, R);

    Vec3 t = Vec3::Zero();

    // test and assert
    MatX P;
    projection_matrix(K, R, t, P);

    EXPECT_EQ(3, P.rows());
    EXPECT_EQ(4, P.cols());
    EXPECT_PRED2(MatricesNear, expected, P);
}

TEST(VisionCommon, convert_single_keypoint) {
    cv::KeyPoint keypoint1(5.f, 5.f, 1.f);
    cv::Point2f keypoint2(1.f, 3.f);
    Vec2 keypoint3(20.0, 22.1);

    Vec2 conv1 = convertKeypoint(keypoint1);
    Vec2 conv2 = convertKeypoint(keypoint2);
    cv::Point2f conv3 = convertKeypoint(keypoint3);

    ASSERT_EQ(keypoint1.pt.x, conv1(0));
    ASSERT_EQ(keypoint1.pt.y, conv1(1));
    ASSERT_EQ(keypoint2.x, conv2(0));
    ASSERT_EQ(keypoint2.y, conv2(1));
    ASSERT_NEAR(keypoint3(0), conv3.x, 0.1);
    ASSERT_NEAR(keypoint3(1), conv3.y, 0.1);
}

TEST(VisionCommon, convert_keypoints) {
    cv::KeyPoint keypoint1(5.f, 5.f, 1.f);
    cv::KeyPoint keypoint2(7.f, 2.f, 1.f);

    cv::Point2f keypoint3(1.f, 8.f);
    cv::Point2f keypoint4(-1.f, 3.0);

    Vec2 keypoint5(9, 2);
    Vec2 keypoint6(1, 2);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::Point2f> keypoints2;
    std::vector<Vec2> keypoints3;

    keypoints1.push_back(keypoint1);
    keypoints1.push_back(keypoint2);

    keypoints2.push_back(keypoint3);
    keypoints2.push_back(keypoint4);

    keypoints3.push_back(keypoint5);
    keypoints3.push_back(keypoint6);

    std::vector<Vec2> conv1 = convertKeypoints(keypoints1);
    std::vector<Vec2> conv2 = convertKeypoints(keypoints2);
    std::vector<cv::Point2f> conv3 = convertKeypoints(keypoints3);

    ASSERT_EQ(conv1.at(0)(0), keypoint1.pt.x);
    ASSERT_EQ(conv1.at(0)(1), keypoint1.pt.y);
    ASSERT_EQ(conv1.at(1)(0), keypoint2.pt.x);
    ASSERT_EQ(conv1.at(1)(1), keypoint2.pt.y);

    ASSERT_EQ(conv2.at(0)(0), keypoint3.x);
    ASSERT_EQ(conv2.at(0)(1), keypoint3.y);
    ASSERT_EQ(conv2.at(1)(0), keypoint4.x);
    ASSERT_EQ(conv2.at(1)(1), keypoint4.y);

    ASSERT_EQ(conv3.at(0).x, keypoints3.at(0)(0));
    ASSERT_EQ(conv3.at(0).y, keypoints3.at(0)(1));
    ASSERT_EQ(conv3.at(1).x, keypoints3.at(1)(0));
    ASSERT_EQ(conv3.at(1).y, keypoints3.at(1)(1));
}

TEST(VisionCommon, read_image_sequence) {
    // Test bad path
    std::string bad_path = "bad_path";
    ASSERT_THROW(readImageSequence(bad_path), std::length_error);

    std::vector<cv::Mat> image_sequence;

    image_sequence = readImageSequence(IMG_FOLDER_PATH);

    ASSERT_EQ((int) image_sequence.size(), 10);
}
}  // namespace wave
