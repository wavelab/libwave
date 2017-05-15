#include "wave/wave_test.hpp"
#include "wave/kinematics/pose.hpp"

namespace wave {

TEST(Pose, constructor) {
    // test default constructor
    Pose p1;

    EXPECT_EQ(1.0, p1.q.w());
    EXPECT_EQ(0.0, p1.q.x());
    EXPECT_EQ(0.0, p1.q.y());
    EXPECT_EQ(0.0, p1.q.z());
    EXPECT_EQ(0.0, p1.position(0));
    EXPECT_EQ(0.0, p1.position(1));
    EXPECT_EQ(0.0, p1.position(2));

    // test quaternion + position constructor
    Quaternion quaternion{1.0, 2.0, 3.0, 4.0};
    Vec3 position{1.0, 2.0, 3.0};
    Pose p2{quaternion, position};

    EXPECT_EQ(1.0, p2.q.w());
    EXPECT_EQ(2.0, p2.q.x());
    EXPECT_EQ(3.0, p2.q.y());
    EXPECT_EQ(4.0, p2.q.z());
    EXPECT_EQ(1.0, p2.position(0));
    EXPECT_EQ(2.0, p2.position(1));
    EXPECT_EQ(3.0, p2.position(2));

    // test euler + position constructor
    Pose p3(0.0, 0.0, 0.0, 4.0, 5.0, 6.0);

    EXPECT_EQ(1.0, p3.q.w());
    EXPECT_EQ(0.0, p3.q.x());
    EXPECT_EQ(0.0, p3.q.y());
    EXPECT_EQ(0.0, p3.q.z());
    EXPECT_EQ(4.0, p3.position(0));
    EXPECT_EQ(5.0, p3.position(1));
    EXPECT_EQ(6.0, p3.position(2));
}

TEST(Pose, rotationMatrix) {
    Pose pose;
    Mat3 R;

    R = pose.rotationMatrix();
    EXPECT_EQ(1.0, R(0, 0));
    EXPECT_EQ(0.0, R(0, 1));
    EXPECT_EQ(0.0, R(0, 2));

    EXPECT_EQ(0.0, R(1, 0));
    EXPECT_EQ(1.0, R(1, 1));
    EXPECT_EQ(0.0, R(1, 2));

    EXPECT_EQ(0.0, R(2, 0));
    EXPECT_EQ(0.0, R(2, 1));
    EXPECT_EQ(1.0, R(2, 2));
}

}  // end of wave namespace
