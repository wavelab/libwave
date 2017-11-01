#include "wave/wave_test.hpp"
#include "wave/vision/utils.hpp"

namespace wave {

const auto IMG_FOLDER_PATH = "tests/data/tracker_test_sequence/frame0057.jpg";

const auto TEST_CONFIG_FILE = "tests/data/config.yaml";

TEST(VisionCommon, focal_length) {
    double f = focal_length(60, 640);
    EXPECT_FLOAT_EQ(554.25629, f);

    Vec2 f2 = focal_length(60, 60, 640, 640);
    EXPECT_FLOAT_EQ(554.25629, f2(0));
    EXPECT_FLOAT_EQ(554.25629, f2(1));
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

class PinholeProjectTest : public ::testing::Test {
 protected:
    // setup camera
    double fx = 554.38;
    double fy = 554.38;
    double cx = 320.0;
    double cy = 320.0;

    Mat3 K;
    PinholeProjectTest() {
        this->K << fx, 0.0, cx,  //
          0.0, fy, cy,           //
          0.0, 0.0, 1.0;
    }

    // rotate the body frame by this to get the camera frame
    // or, use this to transform points from camera frame to body frame
    Quaternion q_BC{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                    Eigen::AngleAxisd(0, Vec3::UnitY()) *
                    Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};
};

TEST_F(PinholeProjectTest, projectPointDirectlyAhead) {
    // Landmark position in global frame
    auto G_p_GF = Vec3{10.0, 0., 0.};

    // Translation from origin of Global frame to Camera
    auto G_p_GC = Vec3{0, 0, 0};

    // Orientation of robot Body frame in Global frame
    auto q_GB = Quaternion::Identity();

    // Get rotation matrix
    auto R_GC = Mat3{q_GB * this->q_BC};

    Vec2 meas;
    auto res = pinholeProject(this->K, R_GC, G_p_GC, G_p_GF, meas);
    auto expected = Vec2{this->cx, this->cy};
    EXPECT_TRUE(res);
    EXPECT_PRED2(VectorsNear, expected, meas);
}

TEST_F(PinholeProjectTest, projectPointDirectlyBehind) {
    // Landmark position in global frame
    auto G_p_GF = Vec3{-10.0, 0., 0.};

    // Translation from origin of Global frame to Camera
    auto G_p_GC = Vec3{0, 0, 0};

    // Orientation of robot Body frame in Global frame
    auto q_GB = Quaternion::Identity();

    // Get rotation matrix
    auto R_GC = Mat3{q_GB * this->q_BC};

    Vec2 meas;
    auto res = pinholeProject(this->K, R_GC, G_p_GC, G_p_GF, meas);
    auto expected = Vec2{this->cx, this->cy};
    EXPECT_FALSE(res);
    EXPECT_PRED2(VectorsNear, expected, meas);
}

TEST_F(PinholeProjectTest, projectPointAboveRight) {
    // Landmark position in global frame
    // It is above and to the right from the camera's viewpoint, since the
    // camera is looking along X, and Y points to the left.
    auto G_p_GF = Vec3{10.0, -2.0, 1.0};

    // Translation from origin of Global frame to Camera
    auto G_p_GC = Vec3{5.0, 0.0, 0.0};

    // Orientation of robot Body frame in Global frame
    auto q_GB = Quaternion::Identity();

    // Get rotation matrix
    auto R_GC = Mat3{q_GB * this->q_BC};

    Vec2 meas;
    auto res = pinholeProject(this->K, R_GC, G_p_GC, G_p_GF, meas);
    EXPECT_TRUE(res);
    EXPECT_GT(meas.x(), this->cx);
    EXPECT_LT(meas.y(), this->cy);
}

TEST_F(PinholeProjectTest, projectKnownPoint) {
    // Test projection with the correct answer separately calculated here

    // Landmark position in global frame
    auto G_p_GF = Vec3{12.0, -3.4, 2.1};

    // Translation from origin of Global frame to Camera
    auto G_p_GC = Vec3{5.0, 0.0, 0.0};

    // Orientation of robot Body frame in Global frame
    auto q_GB = Quaternion::Identity();

    // Get rotation matrix
    auto R_GC = Mat3{q_GB * this->q_BC};

    // Calculate expected
    const Vec3 C_p_CF = R_GC.inverse() * (G_p_GF - G_p_GC);
    const Vec3 homogeneous = this->K * C_p_CF;
    const Vec2 expected = homogeneous.head<2>() / homogeneous(2);

    Vec2 meas;
    auto res = pinholeProject(this->K, R_GC, G_p_GC, G_p_GF, meas);
    EXPECT_TRUE(res);
    EXPECT_PRED2(VectorsNear, expected, meas);
}

TEST(VisionYamlConfig, loadCvMatParam) {
    wave::ConfigParser parser;
    cv::Mat cvmat;

    // setup
    parser.root = YAML::LoadFile(TEST_CONFIG_FILE);
    parser.config_loaded = true;

    // CV MATRIX
    wave::ConfigParam<cv::Mat> cvmat_param{"test_matrix", &cvmat};
    parser.loadParam(cvmat_param);


    for (int i = 0, index = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, cvmat.at<double>(i, j));
            index++;
        }
    }
}

TEST(VisionYamlConfig, cvMatWithConfigParser) {
    cv::Mat cvmat;
    wave::ConfigParser parser;
    parser.addParam("test_matrix", &cvmat);
    parser.load(TEST_CONFIG_FILE);

    for (int i = 0, index = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            ASSERT_FLOAT_EQ((index + 1) * 1.1, cvmat.at<double>(i, j));
            index++;
        }
    }
}

}  // namespace wave
