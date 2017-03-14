#include "wave/wave_test.hpp"
#include "wave/utils/data.hpp"

#define TEST_DATA "tests/data/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"


namespace wave {

TEST(Utils_data_Pose, checkPose) {
    Pose testPose;
    Quaternion q_test;
    Vec3 euler;
    Vec3 position_test;
    Mat3 rotation_mtx;
    Mat3 rotation_mtx_test;

    float roll, pitch, yaw;
    float x, y, z;

    // check Pose with no arguments (q = identity, position = 0);
    ASSERT_FLOAT_EQ(0.0, testPose.orientation.x());
    ASSERT_FLOAT_EQ(0.0, testPose.orientation.y());
    ASSERT_FLOAT_EQ(0.0, testPose.orientation.z());
    ASSERT_FLOAT_EQ(0.0, testPose.orientation.w());

    ASSERT_FLOAT_EQ(0.0, testPose.position(0));
    ASSERT_FLOAT_EQ(0.0, testPose.position(1));
    ASSERT_FLOAT_EQ(0.0, testPose.position(2));

    // test initalizeing with floats,
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    x = 2.0;
    y = 3.0;
    z = 22.0;

    testPose = Pose(roll, pitch, yaw, x, y, z);
    ASSERT_FLOAT_EQ(0, testPose.orientation.x());
    ASSERT_FLOAT_EQ(0, testPose.orientation.y());
    ASSERT_FLOAT_EQ(0, testPose.orientation.z());
    ASSERT_FLOAT_EQ(1, testPose.orientation.w());

    ASSERT_FLOAT_EQ(x, testPose.position(0));
    ASSERT_FLOAT_EQ(y, testPose.position(1));
    ASSERT_FLOAT_EQ(z, testPose.position(2));

    // test initializing non zero roll, pitch, yaw
    roll = 10.0;
    pitch = 15.0;
    yaw = -90.0;

    euler << roll, pitch, yaw;
    euler2quat(euler, 321, q_test);
    testPose = Pose(roll, pitch, yaw, x, y, z);

    ASSERT_FLOAT_EQ(q_test.x(), testPose.orientation.x());
    ASSERT_FLOAT_EQ(q_test.y(), testPose.orientation.y());
    ASSERT_FLOAT_EQ(q_test.z(), testPose.orientation.z());
    ASSERT_FLOAT_EQ(q_test.w(), testPose.orientation.w());

    ASSERT_FLOAT_EQ(x, testPose.position(0));
    ASSERT_FLOAT_EQ(y, testPose.position(1));
    ASSERT_FLOAT_EQ(z, testPose.position(2));

    // test inializing with quaterion and a postion vector
    testPose = Pose(Vec3(x, y, z), q_test);
    ASSERT_FLOAT_EQ(q_test.x(), testPose.orientation.x());
    ASSERT_FLOAT_EQ(q_test.y(), testPose.orientation.y());
    ASSERT_FLOAT_EQ(q_test.z(), testPose.orientation.z());
    ASSERT_FLOAT_EQ(q_test.w(), testPose.orientation.w());

    ASSERT_FLOAT_EQ(x, testPose.position(0));
    ASSERT_FLOAT_EQ(y, testPose.position(1));
    ASSERT_FLOAT_EQ(z, testPose.position(2));

    // test that rotation matrix makes sense
    roll = M_PI / 2;
    pitch = 0.0;
    yaw = M_PI / 2;

    testPose = Pose(roll, pitch, yaw, x, y, z);
    rotation_mtx = testPose.rotationMatrix();
    euler << roll, pitch, yaw;
    euler2quat(euler, 321, q_test);
    ASSERT_TRUE(rotation_mtx == q_test.toRotationMatrix());
}

TEST(Utils_data, csvrows) {
    int rows;
    rows = csvrows(TEST_DATA);
    ASSERT_EQ(281, rows);
}

TEST(Utils_data, csvcols) {
    int cols;
    cols = csvcols(TEST_DATA);
    ASSERT_EQ(2, cols);
}

TEST(Utils_data, csv2mat) {
    MatX data;

    csv2mat(TEST_DATA, true, data);
    ASSERT_EQ(280, data.rows());
    ASSERT_EQ(2, data.cols());
    ASSERT_FLOAT_EQ(-2.22482078596, data(0, 0));
    ASSERT_FLOAT_EQ(9.9625789766, data(0, 1));
    ASSERT_FLOAT_EQ(47.0485650525, data(279, 0));
    ASSERT_FLOAT_EQ(613.503760567, data(279, 1));
}

TEST(Utils_data, mat2csv) {
    MatX x;
    MatX y;

    csv2mat(TEST_DATA, true, x);
    mat2csv(TEST_OUTPUT, x);
    csv2mat(TEST_OUTPUT, false, y);

    for (int i = 0; i < x.rows(); i++) {
        for (int j = 0; j < x.cols(); j++) {
            ASSERT_NEAR(x(i, j), y(i, j), 0.1);
        }
    }
}

}  // end of wave namespace
