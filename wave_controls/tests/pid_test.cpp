#include "wave/wave_test.hpp"
#include "wave/controls/pid.hpp"

namespace wave {

TEST(PID, constructor) {
    PID controller;

    EXPECT_DOUBLE_EQ(0.0, controller.error_prev);
    EXPECT_DOUBLE_EQ(0.0, controller.error_sum);

    EXPECT_DOUBLE_EQ(0.0, controller.error_p);
    EXPECT_DOUBLE_EQ(0.0, controller.error_i);
    EXPECT_DOUBLE_EQ(0.0, controller.error_d);

    EXPECT_DOUBLE_EQ(0.0, controller.k_p);
    EXPECT_DOUBLE_EQ(0.0, controller.k_i);
    EXPECT_DOUBLE_EQ(0.0, controller.k_d);

    controller = PID(1.0, 2.0, 3.0);

    EXPECT_DOUBLE_EQ(0.0, controller.error_prev);
    EXPECT_DOUBLE_EQ(0.0, controller.error_sum);

    EXPECT_DOUBLE_EQ(0.0, controller.error_p);
    EXPECT_DOUBLE_EQ(0.0, controller.error_i);
    EXPECT_DOUBLE_EQ(0.0, controller.error_d);

    EXPECT_DOUBLE_EQ(1.0, controller.k_p);
    EXPECT_DOUBLE_EQ(2.0, controller.k_i);
    EXPECT_DOUBLE_EQ(3.0, controller.k_d);
}

TEST(PID, update) {
    PID controller;
    double output;

    // setup
    controller = PID(1.0, 1.0, 1.0);

    // test and assert
    output = controller.update(10.0, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(1.0, controller.error_sum);
    EXPECT_DOUBLE_EQ(10.0, controller.error_p);
    EXPECT_DOUBLE_EQ(1.0, controller.error_i);
    EXPECT_DOUBLE_EQ(100.0, controller.error_d);
    EXPECT_DOUBLE_EQ(10.0, controller.error_prev);
    EXPECT_DOUBLE_EQ(111.0, output);

    // test and assert
    output = controller.update(10.0, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(2.0, controller.error_sum);
    EXPECT_DOUBLE_EQ(10.0, controller.error_p);
    EXPECT_DOUBLE_EQ(2.0, controller.error_i);
    EXPECT_DOUBLE_EQ(0.0, controller.error_d);
    EXPECT_DOUBLE_EQ(10.0, controller.error_prev);
    EXPECT_DOUBLE_EQ(12.0, output);
}

}  // end of wave namespace
