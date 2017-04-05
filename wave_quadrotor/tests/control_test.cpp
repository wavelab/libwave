#include "wave/wave_test.hpp"
#include "wave/quadrotor/control.hpp"

namespace wave {

TEST(PID, constructor) {
    PID controller;

    ASSERT_FLOAT_EQ(0.0, controller.error_prev);
    ASSERT_FLOAT_EQ(0.0, controller.error_sum);

    ASSERT_FLOAT_EQ(0.0, controller.error_p);
    ASSERT_FLOAT_EQ(0.0, controller.error_i);
    ASSERT_FLOAT_EQ(0.0, controller.error_d);

    ASSERT_FLOAT_EQ(0.0, controller.k_p);
    ASSERT_FLOAT_EQ(0.0, controller.k_i);
    ASSERT_FLOAT_EQ(0.0, controller.k_d);

    controller = PID(1.0, 2.0, 3.0);

    ASSERT_FLOAT_EQ(0.0, controller.error_prev);
    ASSERT_FLOAT_EQ(0.0, controller.error_sum);

    ASSERT_FLOAT_EQ(0.0, controller.error_p);
    ASSERT_FLOAT_EQ(0.0, controller.error_i);
    ASSERT_FLOAT_EQ(0.0, controller.error_d);

    ASSERT_FLOAT_EQ(1.0, controller.k_p);
    ASSERT_FLOAT_EQ(2.0, controller.k_i);
    ASSERT_FLOAT_EQ(3.0, controller.k_d);
}

TEST(PID, calculate) {
    PID controller;
    double output;

    // setup
    controller = PID(1.0, 1.0, 1.0);

    // test and assert
    output = controller.calculate(10.0, 0.0, 0.1);

    ASSERT_FLOAT_EQ(1.0, controller.error_sum);
    ASSERT_FLOAT_EQ(10.0, controller.error_p);
    ASSERT_FLOAT_EQ(1.0, controller.error_i);
    ASSERT_FLOAT_EQ(100.0, controller.error_d);
    ASSERT_FLOAT_EQ(10.0, controller.error_prev);
    ASSERT_FLOAT_EQ(111.0, output);
}

}  // end of wave namespace
