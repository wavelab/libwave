#include "wave/wave_test.hpp"
#include "wave/kinematics/quadrotor.hpp"
#include "wave/controls/pid.hpp"

namespace wave {

#define ATTITUDE_CONTROLLER_OUTPUT "/tmp/quadrotor_attitude_controller.dat"
#define POSITION_CONTROLLER_OUTPUT "/tmp/quadrotor_position_controller.dat"

TEST(AttitudeController, update) {
    double t, dt;
    FILE *output_file;
    QuadrotorModel quad;
    AttitudeController controller;
    Vec4 setpoints;
    Vec4 actual;
    Vec4 motor_inputs;

    // setup
    setpoints << deg2rad(10.0), deg2rad(10.0), deg2rad(10.0), 0.0;
    output_file = fopen(ATTITUDE_CONTROLLER_OUTPUT, "w");
    fprintf(output_file, "t, roll, pitch, yaw\n");

    // control attitude
    t = 0;
    dt = 0.001;
    for (int i = 0; i < 1000; i++) {
        actual << quad.states(0), quad.states(1), quad.states(2),
          quad.states(8);
        motor_inputs = controller.update(setpoints, actual, dt);
        quad.update(motor_inputs, dt);

        fprintf(output_file, "%f, ", t);
        fprintf(output_file, "%f, ", rad2deg(quad.states(0)));
        fprintf(output_file, "%f, ", rad2deg(quad.states(1)));
        fprintf(output_file, "%f\n", rad2deg(quad.states(2)));
        t += dt;
    }

    // clean up
    fclose(output_file);
}

TEST(PositionController, update) {
    double t, dt;
    FILE *output_file;
    QuadrotorModel quad;
    PositionController pos_controller;
    AttitudeController att_controller;
    Vec3 pos_setpoints;
    Vec4 att_setpoints;
    Vec4 actual_position;
    Vec4 actual_attitude;
    Vec4 motor_inputs;

    // setup
    pos_setpoints << 1.0, 0.0, 2.0;
    output_file = fopen(POSITION_CONTROLLER_OUTPUT, "w");
    fprintf(output_file, "t, roll, pitch, yaw, x, y, z\n");
    quad.states(2) = deg2rad(50.0);

    // control position
    t = 0;
    dt = 0.001;
    for (int i = 0; i < 5000; i++) {
        // position controller
        // clang-format off
        actual_position << quad.states(6),
                           quad.states(7),
                           quad.states(8),
                           quad.states(2);
        att_setpoints = pos_controller.update(
            pos_setpoints,
            actual_position,
            deg2rad(10.0),
            dt
        );
        // clang-format on

        // attitude controller
        // clang-format off
        actual_attitude << quad.states(0),
                           quad.states(1),
                           quad.states(2),
                           quad.states(8);
        motor_inputs = att_controller.update(
            att_setpoints,
            actual_attitude,
            dt
        );
        quad.update(motor_inputs, dt);
        // clang-format on

        // record quadrotor state
        fprintf(output_file, "%f, ", t);
        fprintf(output_file, "%f, ", rad2deg(quad.states(0)));
        fprintf(output_file, "%f, ", rad2deg(quad.states(1)));
        fprintf(output_file, "%f, ", rad2deg(quad.states(2)));
        fprintf(output_file, "%f, ", quad.states(6));
        fprintf(output_file, "%f, ", quad.states(7));
        fprintf(output_file, "%f\n", quad.states(8));
        t += dt;
    }

    fclose(output_file);
}

}  // end of wave namespace
