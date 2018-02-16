/* Copyright (c) 2017, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: test_angles.cpp
 * Desc: Tests for angle wrapping functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *       Pranav Ganti <nav.ganti@uwaterloo.ca>
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include "wave/utils/angles.hpp"

namespace wave {

TEST(AnglesTest, WrapToPiTest) {
    auto test_angle_1 = 0;
    auto test_angle_2 = M_PI / 2;
    auto test_angle_3 = M_PI;
    auto test_angle_4 = 3 * M_PI / 2;
    auto test_angle_5 = 2 * M_PI;
    auto test_angle_6 = -M_PI / 2;
    auto test_angle_7 = -M_PI;
    auto test_angle_8 = -3 * M_PI / 2;
    auto test_angle_9 = -2 * M_PI;
    auto test_angle_10 = 3 * M_PI;
    auto test_angle_11 = -3 * M_PI;
    auto test_angle_12 = 12 * M_PI;
    auto test_angle_13 = -12 * M_PI;
    auto test_angle_14 = 13 * M_PI;
    auto test_angle_15 = -13 * M_PI;

    ASSERT_EQ(0, wrapToPi(test_angle_1));
    ASSERT_EQ(M_PI / 2, wrapToPi(test_angle_2));
    ASSERT_EQ(-M_PI, wrapToPi(test_angle_3));
    ASSERT_EQ(-M_PI / 2, wrapToPi(test_angle_4));
    ASSERT_EQ(0, wrapToPi(test_angle_5));
    ASSERT_EQ(-M_PI / 2, wrapToPi(test_angle_6));
    ASSERT_EQ(-M_PI, wrapToPi(test_angle_7));
    ASSERT_EQ(M_PI / 2, wrapToPi(test_angle_8));
    ASSERT_EQ(0, wrapToPi(test_angle_9));
    ASSERT_EQ(-M_PI, wrapToPi(test_angle_10));
    ASSERT_EQ(-M_PI, wrapToPi(test_angle_11));
    ASSERT_NEAR(0, wrapToPi(test_angle_12), 1e-12);
    ASSERT_NEAR(0, wrapToPi(test_angle_13), 1e-12);
    ASSERT_NEAR(-M_PI, wrapToPi(test_angle_14), 1e-12);
    ASSERT_NEAR(-M_PI, wrapToPi(test_angle_15), 1e-12);
}

TEST(AnglesTest, WrapToTwoPiTest) {
    auto test_angle_1 = 0;
    auto test_angle_2 = M_PI / 2;
    auto test_angle_3 = M_PI;
    auto test_angle_4 = 3 * M_PI / 2;
    auto test_angle_5 = 2 * M_PI;
    auto test_angle_6 = -M_PI / 2;
    auto test_angle_7 = -M_PI;
    auto test_angle_8 = -3 * M_PI / 2;
    auto test_angle_9 = -2 * M_PI;
    auto test_angle_10 = 3 * M_PI;
    auto test_angle_11 = -3 * M_PI;
    auto test_angle_12 = 12 * M_PI;
    auto test_angle_13 = -12 * M_PI;
    auto test_angle_14 = 13 * M_PI;
    auto test_angle_15 = -13 * M_PI;

    ASSERT_EQ(0, wrapToTwoPi(test_angle_1));
    ASSERT_EQ(M_PI / 2, wrapToTwoPi(test_angle_2));
    ASSERT_EQ(M_PI, wrapToTwoPi(test_angle_3));
    ASSERT_EQ(2 * M_PI - M_PI / 2, wrapToTwoPi(test_angle_4));
    ASSERT_EQ(0, wrapToTwoPi(test_angle_5));
    ASSERT_EQ(2 * M_PI - M_PI / 2, wrapToTwoPi(test_angle_6));
    ASSERT_EQ(M_PI, wrapToTwoPi(test_angle_7));
    ASSERT_EQ(M_PI / 2, wrapToTwoPi(test_angle_8));
    ASSERT_EQ(0, wrapToTwoPi(test_angle_9));
    ASSERT_EQ(M_PI, wrapToTwoPi(test_angle_10));
    ASSERT_EQ(M_PI, wrapToTwoPi(test_angle_11));
    ASSERT_NEAR(0, wrapToTwoPi(test_angle_12), 1e-12);
    ASSERT_NEAR(0, wrapToTwoPi(test_angle_13), 1e-12);
    ASSERT_NEAR(M_PI, wrapToTwoPi(test_angle_14), 1e-12);
    ASSERT_NEAR(M_PI, wrapToTwoPi(test_angle_15), 1e-12);
}

}  // namespace wave
