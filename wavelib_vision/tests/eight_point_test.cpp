#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "slam/utils/utils.hpp"
#include "slam/utils/math.hpp"

#include "slam/vision/vo.hpp"
#include "slam/vision/good.hpp"
#include "slam/vision/fast.hpp"
#include "slam/vision/utils.hpp"
#include "slam/vision/eight_point.hpp"

#define TEST_1_DATA "tests/data/eight_point/img_1.dat"
#define TEST_2_DATA "tests/data/eight_point/img_2.dat"
#define TEST_IMAGE_1 "tests/data/eight_point/left.jpg"
#define TEST_IMAGE_2 "tests/data/eight_point/right.jpg"


slam::MatX load_data(std::string file_path)
{
    int line_no;
    int nb_lines;
    std::string line;
    std::ifstream infile(file_path);
    std::vector<double> vdata;
    std::string element;
    double value;
    slam::MatX data;

    // load file
    if (infile.good() != true) {
        printf("ERROR: FAILED TO LOAD TEST DATA [%s]!!\n", file_path.c_str());
        exit(-1);
    }

    // obtain number of lines
    nb_lines = 0;
    while (std::getline(infile, line)) {
        nb_lines++;
    }

    // rewind file
    infile.clear();
    infile.seekg(0);

    // parse file line by line
    line_no = 0;
    data.resize(nb_lines - 1, 3);
    while (std::getline(infile, line)) {
        std::istringstream ss(line);

        if (line_no != 0) {  // skip header line
            // assuming data is 2 columns
            for (int i = 0; i < 2; i++) {
                std::getline(ss, element, ',');
                value = atof(element.c_str());
                data(line_no - 1, i) = value;
            }
            data(line_no - 1, 2) = 1;
        }

        line_no++;
    }

    return data;
}

TEST(EightPoint, constructor)
{
    slam::optimization::EightPoint estimator;

    ASSERT_EQ(false, estimator.configured);

    ASSERT_EQ(0, estimator.image_width);
    ASSERT_EQ(0, estimator.image_height);
}

TEST(EightPoint, configure)
{
    slam::optimization::EightPoint estimator;

    // setup
    estimator.configure(800, 600);

    // test and assert
    ASSERT_EQ(true, estimator.configured);

    ASSERT_EQ(800, estimator.image_width);
    ASSERT_EQ(600, estimator.image_height);
}

TEST(EightPoint, normalizePoints)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);

    // test and assert
    estimator.normalizePoints(pts1, pts2);
    for (int i = 0; i < pts1.rows(); i++) {
        ASSERT_TRUE(pts1(i, 0) < 1.0);
        ASSERT_TRUE(pts1(i, 0) > -1.0);
        ASSERT_TRUE(pts1(i, 1) < 1.0);
        ASSERT_TRUE(pts1(i, 1) > -1.0);

        ASSERT_TRUE(pts2(i, 0) < 1.0);
        ASSERT_TRUE(pts2(i, 0) > -1.0);
        ASSERT_TRUE(pts2(i, 1) < 1.0);
        ASSERT_TRUE(pts2(i, 1) > -1.0);
    }
}

TEST(EightPoint, formMatrixA)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);

    // test and assert
    estimator.formMatrixA(pts1, pts2, A);

    ASSERT_EQ(pts1.rows(), A.rows());
    ASSERT_EQ(9, A.cols());

    for (int i = 0; i < pts1.rows(); i++) {
        ASSERT_FLOAT_EQ(pts1(i, 0) * pts2(i, 0), A(i, 0));
        ASSERT_FLOAT_EQ(pts1(i, 1) * pts2(i, 0), A(i, 1));
        ASSERT_FLOAT_EQ(pts2(i, 0), A(i, 2));
        ASSERT_FLOAT_EQ(pts1(i, 0) * pts2(i, 1), A(i, 3));
        ASSERT_FLOAT_EQ(pts1(i, 1) * pts2(i, 1), A(i, 4));
        ASSERT_FLOAT_EQ(pts2(i, 1), A(i, 5));
        ASSERT_FLOAT_EQ(pts1(i, 0), A(i, 6));
        ASSERT_FLOAT_EQ(pts1(i, 1), A(i, 7));
        ASSERT_FLOAT_EQ(1.0, A(i, 8));
    }
}

TEST(EightPoint, approximateFundamentalMatrix)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);

    // test and assert
    estimator.approximateFundamentalMatrix(A, F);
    ASSERT_EQ(3, F.rows());
    ASSERT_EQ(3, F.cols());
}

TEST(EightPoint, refineFundamentalMatrix)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);
    estimator.approximateFundamentalMatrix(A, F);

    // test and assert
    estimator.refineFundamentalMatrix(F);
    ASSERT_EQ(3, F.rows());
    ASSERT_EQ(3, F.cols());
}

TEST(EightPoint, denormalizeFundamentalMatrix)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);
    estimator.approximateFundamentalMatrix(A, F);
    estimator.refineFundamentalMatrix(F);

    // test and assert
    estimator.denormalizeFundamentalMatrix(F);
}

TEST(EightPoint, estimate)
{
    double result;
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX F;
    slam::Mat3 K, E;
    slam::Vec3 x;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    K << 687.189819, 0.000000, 375.042664,
         0.000000, 641.376221, 308.712708,
         0.000000, 0.000000, 1.000000;

    // estimate fundamental matrix F
    estimator.estimate(pts1, pts2, F);

    // for (int i = 0; i < pts1.rows(); i++) {
    //     x = pts1.block(i, 0, 1, 3).transpose();
    //     result = x.transpose() * F * x;
    //     mu_check(result > 0.0);
    //     mu_check(result < 1.0);
    // }

    // for (int i = 0; i < pts2.rows(); i++) {
    //     x = pts2.block(i, 0, 1, 3).transpose();
    //     result = x.transpose() * F * x;
    //     mu_check(result > 0.0);
    //     mu_check(result < 1.0);
    // }

    // estimate essential matrix E
    estimator.estimate(pts1, pts2, K, E);
}

TEST(EightPoint, obtainPossiblePoses)
{
    double result;
    slam::MatX pts1;
    slam::MatX pts2;
    slam::Mat3 K, E;
    slam::optimization::EightPoint estimator;
    std::vector<slam::MatX> poses;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    K << 687.189819, 0.000000, 375.042664,
         0.000000, 641.376221, 308.712708,
         0.000000, 0.000000, 1.000000;
    estimator.estimate(pts1, pts2, K, E);

    // test and assert
    estimator.obtainPossiblePoses(E, poses);

    std::cout << poses[0] << std::endl;
    std::cout << std::endl;
    std::cout << poses[1] << std::endl;
    std::cout << std::endl;
    std::cout << poses[2] << std::endl;
    std::cout << std::endl;
    std::cout << poses[3] << std::endl;
}

TEST(EightPoint, obtainPose)
{
    double result;
    slam::MatX pts1;
    slam::MatX pts2;
    slam::Vec3 pt1;
    slam::Vec3 pt2;
    slam::Mat3 K, E;
    slam::MatX pose;
    slam::optimization::EightPoint estimator;
    std::vector<slam::MatX> poses;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    K << 687.189819, 0.000000, 375.042664,
         0.000000, 641.376221, 308.712708,
         0.000000, 0.000000, 1.000000;
    estimator.estimate(pts1, pts2, K, E);
    estimator.obtainPossiblePoses(E, poses);

    // test and assert
    pt1 = pts1.block(0, 0, 1, 3).transpose();
    pt2 = pts2.block(0, 0, 1, 3).transpose();
    estimator.obtainPose(pt1, pt2, K, K, poses, pose);
    std::cout << "8 point: \n" << pose << std::endl << std::endl;

    // compare with opencv's 8 point
    slam::VisualOdometry vo;
    vo.configure(K);
    std::vector<cv::Point2f> cvpts1;
    std::vector<cv::Point2f> cvpts2;

    for (int i = 0; i < pts1.rows(); i++) {
        cvpts1.push_back(cv::Point2f(pts1(i, 0), pts1(i, 1)));
        cvpts2.push_back(cv::Point2f(pts2(i, 0), pts2(i, 1)));
    }
    vo.measure(cvpts1, cvpts2);
    std::cout << "cv rotation: \n" << vo.R << std::endl << std::endl;
    std::cout << "cv translation: \n" << vo.t << std::endl << std::endl;
}

TEST(EightPoint, obtainPose2)
{
    double result;
    cv::Mat img_1, img_2, twin_img;
    std::vector<cv::Point2f> cvpts_1, cvpts_2;
    std::vector<float> errors;
    std::vector<uchar> status;

    slam::MatX pts1, pts2;
    slam::Vec3 pt1, pt2;
    slam::Mat3 K, E;
    slam::MatX pose;
    std::vector<slam::MatX> poses;
    slam::optimization::EightPoint estimator;
    slam::FastDetector fast;
    slam::VisualOdometry vo;

    // setup
    K << 279.0161682343449, 0, 150.3072895826164,
         0, 276.3467561622266, 123.3623526538343,
         0, 0, 1;
    fast.configure(40, true);
    vo.configure(K);

    // load test image
    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);

    // detect and track features
    fast.detect(img_1, cvpts_1);
    vo.featureTracking(img_1, img_2, cvpts_1, cvpts_2, errors, status);
    slam::cvpts2mat(cvpts_1, pts1);
    slam::cvpts2mat(cvpts_2, pts2);

    // display tracked features
    vo.drawOpticalFlow(img_1, img_2, cvpts_1, cvpts_2, twin_img);
    cv::imshow("image", twin_img);
    cv::waitKey(5000);

    // test 8-point
    estimator.configure(960, 720);
    estimator.estimate(pts1, pts2, K, E);
    estimator.obtainPossiblePoses(E, poses);

    pt1 = pts1.block(0, 0, 1, 3).transpose();
    pt2 = pts2.block(0, 0, 1, 3).transpose();
    estimator.obtainPose(pt1, pt2, K, K, poses, pose);
    std::cout << "8 point: \n" << pose << std::endl << std::endl;

    // compare with opencv 8-point
    vo.measure(cvpts_1, cvpts_2);
    std::cout << "cv rotation: \n" << vo.R << std::endl << std::endl;
    std::cout << "cv translation: \n" << vo.t << std::endl << std::endl;
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
