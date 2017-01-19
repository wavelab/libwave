#include <gtest/gtest.h>

#include "wavelib/utils/utils.hpp"
#include "wavelib/optimization/testcase.hpp"


TEST(Testcase, constructor)
{
    wavelib::TestCase testcase;

    ASSERT_EQ(false, testcase.configured);
}

TEST(Testcase, configure)
{
    wavelib::TestCase testcase;

    testcase.configure();
    ASSERT_EQ(true, testcase.configured);
}

TEST(TestCase, createP)
{
    wavelib::TestCase testcase;
    wavelib::Mat3 K, R;
    wavelib::MatX P;
    wavelib::Vec3 t;

    // setup
    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    R << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    t << 1.0, 2.0, 3.0;

    // test and assert
    testcase.createP(K, R, t, P);
    // std::cout << P << std::endl;

    ASSERT_FLOAT_EQ(1.0, P(0, 0));
    ASSERT_FLOAT_EQ(1.0, P(1, 1));
    ASSERT_FLOAT_EQ(1.0, P(2, 2));

    ASSERT_FLOAT_EQ(-1.0, P(0, 3));
    ASSERT_FLOAT_EQ(-2.0, P(1, 3));
    ASSERT_FLOAT_EQ(-3.0, P(2, 3));
}

TEST(Testcase, generateRandom3DPoints)
{
    wavelib::TestCase testcase;
    wavelib::TestRange range;
    wavelib::MatX pts;

    range.x_min = 0.0;
    range.x_max = 1.0;

    range.y_min = 0.0;
    range.y_max = 1.0;

    range.z_min = 0.1;
    range.z_max = 1.0;

    testcase.configure();
    testcase.generateRandom3DPoints(range, 10, pts);

    for (int i = 0; i < 10; i++) {
        ASSERT_TRUE(pts(i, 0) > 0.0);
        ASSERT_TRUE(pts(i, 0) < 1.0);

        ASSERT_TRUE(pts(i, 1) > 0.0);
        ASSERT_TRUE(pts(i, 1) < 1.0);

        ASSERT_TRUE(pts(i, 2) > 0.0);
        ASSERT_TRUE(pts(i, 2) < 1.0);
    }
}

TEST(TestCase, project3DTo2D)
{
    wavelib::TestRange range;
    wavelib::TestCase testcase;
    wavelib::MatX pts_3d;
    wavelib::MatX pts_2d;
    wavelib::Mat3 K, R;
    wavelib::Vec3 t;

    // setup
    range.x_min = -1.0;
    range.x_max = 1.0;

    range.y_min = -1.0;
    range.y_max = 1.0;

    range.z_min = 2.0;
    range.z_max = 5.0;

    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    R << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    t << 0.0, 0.0, 0.0;

    testcase.generateRandom3DPoints(range, 10, pts_3d);
    testcase.project3DTo2D(K, R, t, pts_3d, pts_2d);
}

TEST(TestCase, generateTestCase)
{
    wavelib::TestRange range;
    wavelib::TestCase testcase;
    wavelib::MatX pts1, pts2, pts3d;

    // setup
    range.x_min = -1.0;
    range.x_max = 1.0;

    range.y_min = -1.0;
    range.y_max = 1.0;

    range.z_min = 2.0;
    range.z_max = 5.0;

    testcase.generateTestCase(range, pts1, pts2, pts3d);

    wavelib::mat2csv("/tmp/pts1.dat", pts1);
    wavelib::mat2csv("/tmp/pts2.dat", pts2);
    wavelib::mat2csv("/tmp/pts3d.dat", pts3d);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
