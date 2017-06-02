#include "wave/wave_test.hpp"
#include "wave/kinematics/pose.hpp"

namespace wave {

TEST(Pose, constructor) {
    // test default constructor
    Pose p1;
    EXPECT_TRUE(p1.position.isApprox(Vec3{0.0, 0.0, 0.0}));
    EXPECT_TRUE(p1.orientation.isApprox(Quaternion{1.0, 0.0, 0.0, 0.0}));

    // test quaternion + position constructor
    Quaternion orientation{1.0, 2.0, 3.0, 4.0};
    Vec3 position{1.0, 2.0, 3.0};
    Pose p2{position, orientation};
    EXPECT_TRUE(p2.position.isApprox(Vec3{1.0, 2.0, 3.0}));
    EXPECT_TRUE(p2.orientation.isApprox(Quaternion{1.0, 2.0, 3.0, 4.0}));

    // test euler + position constructor
    Pose p3(0.0, 0.0, 0.0, 4.0, 5.0, 6.0);
    EXPECT_TRUE(p3.position.isApprox(Vec3{4.0, 5.0, 6.0}));
    EXPECT_TRUE(p3.orientation.isApprox(Quaternion{1.0, 0.0, 0.0, 0.0}));
}

TEST(Pose, rotationMatrix) {
    Pose pose;
    Mat3 R;

    R = pose.rotationMatrix();
    EXPECT_TRUE(R.isApprox(Mat3::Identity()));
}

}  // namespace wave
