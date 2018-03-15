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
 * File: test_ecef_llh_point_conversions.cpp
 * Desc: Tests for ECEF <-> LLH conversions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include "wave/geography/world_frame_conversions.hpp"

namespace wave {

TEST(ecefAndLLHPointConversionTest, everestTest) {
    // From GeographicLib's example:
    // Mt Everest
    //      LLH[deg, deg, m] ~ (27.99, 86.93, 8820)
    //      ECEF[km] ~ (302.3, 5636, 2980)
    //      ECEF[m] ~ (3.023e5, 5.636e6, 2.980e6)

    double point_llh[3] = {27.99, 86.93, 8820};
    double point_ECEF[3] = {3.023e5, 5.636e6, 2.980e6};

    // LLH to ECEF
    double point_ECEF_Results[3];
    ecefPointFromLLH(point_llh, point_ECEF_Results);

    // Precisions are based on the sig-fig data above -> therefore different
    // checks have different bounds based on their order of magnitude.
    ASSERT_NEAR(point_ECEF[0], point_ECEF_Results[0], 1e2);
    ASSERT_NEAR(point_ECEF[1], point_ECEF_Results[1], 1e3);
    ASSERT_NEAR(point_ECEF[2], point_ECEF_Results[2], 1e3);

    // ECEF back to LLH
    double point_llh_results[3];
    llhPointFromECEF(point_ECEF_Results, point_llh_results);

    ASSERT_NEAR(point_llh[0], point_llh_results[0], 1e-2);
    ASSERT_NEAR(point_llh[1], point_llh_results[1], 1e-2);
    ASSERT_NEAR(point_llh[2], point_llh_results[2], 1.0);
}

TEST(ecefAndLLHPointConversionTest, grandCanyonTest) {
    // Second test for a lower point located elsewhere:
    // Badwater Basin
    //      LLH[deg, deg, m] ~ (36.250278, -116.825833, -85.9536)
    //      ECEF[km] ~ (-2323.892, -4595.374, 3750.572)
    //      ECEF[m] ~ (-2.323892e6, -4.595374e6, 3.750572e6)

    double point_llh[3] = {36.250278, -116.825833, -85.9536};
    double point_ECEF[3] = {-2.323892e6, -4.595374e6, 3.750572e6};

    double point_ECEF_Results[3];
    ecefPointFromLLH(point_llh, point_ECEF_Results);

    // Higher precision inputs should have tighter check
    ASSERT_NEAR(point_ECEF[0], point_ECEF_Results[0], 1e1);
    ASSERT_NEAR(point_ECEF[1], point_ECEF_Results[1], 1e1);
    ASSERT_NEAR(point_ECEF[2], point_ECEF_Results[2], 1e1);

    // ECEF back to LLH
    double point_llh_results[3];
    llhPointFromECEF(point_ECEF_Results, point_llh_results);

    ASSERT_NEAR(point_llh[0], point_llh_results[0], 1e-2);
    ASSERT_NEAR(point_llh[1], point_llh_results[1], 1e-2);
    ASSERT_NEAR(point_llh[2], point_llh_results[2], 1.0);
}

}  // namespace wave
