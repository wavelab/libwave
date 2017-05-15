#include "wave/wave_test.hpp"
#include "wave/kinematics/pose.hpp"

namespace wave {

TEST(Pose, constructor) {
    // test default constructor
    Pose p1;

    EXPECT_EQ(0.0, p1.position(0));
    EXPECT_EQ(0.0, p1.position(1));
    EXPECT_EQ(0.0, p1.position(2));
    EXPECT_EQ(1.0, p1.orientation.w());
    EXPECT_EQ(0.0, p1.orientation.x());
    EXPECT_EQ(0.0, p1.orientation.y());
    EXPECT_EQ(0.0, p1.orientation.z());

    // test quaternion + position constructor
    Quaternion orientation{1.0, 2.0, 3.0, 4.0};
    Vec3 position{1.0, 2.0, 3.0};
    Pose p2{position, orientation};

    EXPECT_EQ(1.0, p2.position(0));
    EXPECT_EQ(2.0, p2.position(1));
    EXPECT_EQ(3.0, p2.position(2));
    EXPECT_EQ(1.0, p2.orientation.w());
    EXPECT_EQ(2.0, p2.orientation.x());
    EXPECT_EQ(3.0, p2.orientation.y());
    EXPECT_EQ(4.0, p2.orientation.z());

    // test euler + position constructor
    Pose p3(0.0, 0.0, 0.0, 4.0, 5.0, 6.0);

    EXPECT_EQ(4.0, p3.position(0));
    EXPECT_EQ(5.0, p3.position(1));
    EXPECT_EQ(6.0, p3.position(2));
    EXPECT_EQ(1.0, p3.orientation.w());
    EXPECT_EQ(0.0, p3.orientation.x());
    EXPECT_EQ(0.0, p3.orientation.y());
    EXPECT_EQ(0.0, p3.orientation.z());
}

TEST(Pose, rotationMatrix) {
    Pose pose;
    Mat3 R;

    R = pose.rotationMatrix();
    EXPECT_TRUE(R.isApprox(Mat3::Identity()));
}

}  // end of wave namespace
