#include "wave/wave_test.hpp"
#include "wave/kinematics/two_wheel.hpp"

namespace wave {

TEST(TwoWheelRobot2DModel, update) {
    // setup
    TwoWheelRobot2DModel robot;
    double dt = 0.01;
    Vec2 motor_inputs;
    motor_inputs << 1.0, 0.0;

    // simulate and assert
    for (int i = 0; i < 100; i++) {
        robot.update(motor_inputs, dt);
    }

    EXPECT_TRUE(robot.pose.isApprox(Vec3(1.0, 0.0, 0.0)));
}

}  // end of wave namespace
