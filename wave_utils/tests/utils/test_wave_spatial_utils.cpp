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
 * File: test_wave_spatial_utils.cpp
 * Desc: Main function for testing wave_spatial_utils package.
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *       Pranav Ganti <nav.ganti@uwaterloo.ca>
 *
 * ############################################################################
*/

#include "gtest/gtest.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
