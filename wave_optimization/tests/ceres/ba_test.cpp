#include "wave/wave_test.hpp"
#include "wave/utils/utils.hpp"
#include "wave/optimization/ceres/ba.hpp"

#define TEST_DATA_1 "tests/data/optimization/ceres/ba/pts1.dat"
#define TEST_DATA_2 "tests/data/optimization/ceres/ba/pts2.dat"
#define TEST_DATA_3 "tests/data/optimization/ceres/ba/pts3d.dat"

namespace wave {

TEST(BAResidual, constructor) {
    Mat3 K;
    Vec2 x;

    // setup
    // clang-format off
    K << 1.0, 0.0, 3.0,
         0.0, 2.0, 4.0,
         0.0, 0.0, 1.0;
    x << 130, 62;
    // clang-format on

    // test
    BAResidual r(K, x, true);
    EXPECT_FLOAT_EQ(K(0, 0), r.fx);
    EXPECT_FLOAT_EQ(K(1, 1), r.fy);
    EXPECT_FLOAT_EQ(K(0, 2), r.cx);
    EXPECT_FLOAT_EQ(K(1, 2), r.cy);
    EXPECT_FLOAT_EQ(x(0), r.x);
    EXPECT_FLOAT_EQ(x(1), r.y);
    EXPECT_FLOAT_EQ(true, r.origin);
}

TEST(BundleAdjustment, constructor) {
    // test default constructor
    BundleAdjustment ba1;

    EXPECT_TRUE(ba1.K.isApprox(Mat3::Zero(3, 3)));
    EXPECT_TRUE(ba1.x1_pts.isApprox(MatX::Zero(1, 3)));
    EXPECT_TRUE(ba1.x2_pts.isApprox(MatX::Zero(1, 3)));
    EXPECT_EQ(NULL, ba1.q);
    EXPECT_EQ(NULL, ba1.c);
    EXPECT_EQ(NULL, ba1.x);

    // test construct with arguments
    Mat3 K;
    MatX x1_pts, x2_pts;

    // clang-format off
    K << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
    x1_pts = MatX::Random(100, 3);
    x2_pts = MatX::Random(100, 3);
    // clang-format on

    BundleAdjustment ba2{K, x1_pts, x2_pts};
}

// TEST(BundleAdjustment, solve) {
//     Mat3 K;
//     MatX x1_pts, x2_pts, pts3d;
//     BundleAdjustment ba;
//     struct timespec t;
//
//     // setup
//     csv2mat(TEST_DATA_1, false, x1_pts);
//     csv2mat(TEST_DATA_2, false, x2_pts);
//     csv2mat(TEST_DATA_3, false, pts3d);
//
//     K << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
//
//     tic(&t);
//     ba = BundleAdjustment(K, x1_pts, x2_pts);
//     std::cout << mtoc(&t) << " ms" << std::endl << std::endl;
//
//     // test
//     tic(&t);
//     ba.solve(pts3d);
//     std::cout << mtoc(&t) << " ms" << std::endl << std::endl;
//
//     std::cout << "q: (";
//     std::cout << ba.q[1][0] << ", ";
//     std::cout << ba.q[1][1] << ", ";
//     std::cout << ba.q[1][2] << ", ";
//     std::cout << ba.q[1][3] << ")";
//     std::cout << std::endl;
//
//     std::cout << "c: (";
//     std::cout << ba.c[1][0] << ", ";
//     std::cout << ba.c[1][1] << ", ";
//     std::cout << ba.c[1][2] << ")";
//     std::cout << std::endl;
// }

}  // end of wave namespace
