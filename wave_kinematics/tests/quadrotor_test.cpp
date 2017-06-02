#include "wave/wave_test.hpp"
#include "wave/kinematics/quadrotor.hpp"
#include "wave/controls/pid.hpp"

namespace wave {

#define ATTITUDE_CONTROLLER_OUTPUT "/tmp/quadrotor_attitude_controller.dat"
#define POSITION_CONTROLLER_OUTPUT "/tmp/quadrotor_position_controller.dat"

TEST(AttitudeController, update) {
    QuadrotorModel quad;
    AttitudeController controller;

    // setup
    Vec4 setpoints;
    setpoints << deg2rad(10.0), deg2rad(10.0), deg2rad(10.0), 0.0;
    FILE *output_file = fopen(ATTITUDE_CONTROLLER_OUTPUT, "w");
    fprintf(output_file, "t, roll, pitch, yaw\n");

    // control attitude
    double t = 0;
    double dt = 0.001;
    Vec4 actual;
    Vec4 motor_inputs;

    for (int i = 0; i < 1000; i++) {
        // clang-format off
        actual << quad.attitude(0),
                  quad.attitude(1),
                  quad.attitude(2),
                  quad.position(2);
        // clang-format on
        motor_inputs = controller.update(setpoints, actual, dt);
        quad.update(motor_inputs, dt);

        fprintf(output_file, "%f, ", t);
        fprintf(output_file, "%f, ", rad2deg(quad.position(0)));
        fprintf(output_file, "%f, ", rad2deg(quad.position(1)));
        fprintf(output_file, "%f\n", rad2deg(quad.position(2)));
        t += dt;
    }

    // clean up
    fclose(output_file);

    // assert
    ASSERT_NEAR(setpoints(0), quad.attitude(0), 0.01);
    ASSERT_NEAR(setpoints(1), quad.attitude(1), 0.01);
    ASSERT_NEAR(setpoints(2), quad.attitude(2), 0.01);
}

TEST(PositionController, update) {
    QuadrotorModel quad;
    PositionController pos_controller;
    AttitudeController att_controller;

    // setup
    Vec3 pos_setpoints;
    pos_setpoints << 1.0, 0.0, 2.0;

    FILE *output_file = fopen(POSITION_CONTROLLER_OUTPUT, "w");
    fprintf(output_file, "t, roll, pitch, yaw, x, y, z\n");

    quad.position << 0.0, 0.0, 0.0;

    // control position
    double t = 0;
    double dt = 0.001;
    Vec4 actual_position;
    Vec4 att_setpoints;
    Vec4 actual_attitude;
    Vec4 motor_inputs;

    for (int i = 0; i < 5000; i++) {
        // position controller
        // clang-format off
        actual_position << quad.position(0),
                           quad.position(1),
                           quad.position(2),
                           quad.attitude(2);
        att_setpoints = pos_controller.update(
            pos_setpoints,
            actual_position,
            deg2rad(10.0),
            dt
        );
        // clang-format on

        // attitude controller
        // clang-format off
        actual_attitude << quad.attitude(0),
                           quad.attitude(1),
                           quad.attitude(2),
                           quad.position(2);
        motor_inputs = att_controller.update(
            att_setpoints,
            actual_attitude,
            dt
        );
        quad.update(motor_inputs, dt);
        // clang-format on

        // record quadrotor state
        fprintf(output_file, "%f, ", t);
        fprintf(output_file, "%f, ", rad2deg(quad.position(0)));
        fprintf(output_file, "%f, ", rad2deg(quad.position(1)));
        fprintf(output_file, "%f, ", rad2deg(quad.position(2)));
        fprintf(output_file, "%f, ", quad.position(0));
        fprintf(output_file, "%f, ", quad.position(1));
        fprintf(output_file, "%f\n", quad.position(2));
        t += dt;
    }
    fclose(output_file);

    // assert
    ASSERT_NEAR(pos_setpoints(0), quad.position(0), 0.01);
    ASSERT_NEAR(pos_setpoints(1), quad.position(1), 0.01);
    ASSERT_NEAR(pos_setpoints(2), quad.position(2), 0.01);
}

}  // namespace wave
