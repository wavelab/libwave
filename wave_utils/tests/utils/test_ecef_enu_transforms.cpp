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
 * File: test_ecef_enu_transforms.cpp
 * Desc: Tests for ECEF <-> ENU transform matrices
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include "wave/utils/world_frame_conversions.hpp"

namespace wave {

class ECEFtoENUTest : public ::testing::Test {
 protected:
    double near90 = 90.0 - 1e-6;
    double near180 = 180.0 - 1e-6;
    double rotation_check_threshold = 1e-6;
    double earth_radius_approx = 6.371e6;  // +/- 21km between WGS84 max/min

    void checkT_ECEF_ENU(double T_ECEF_ENU[4][4],
                         double expected_R_ENU_ECEF[3][3]) {
        // Compare R vs R^T
        for (int i_row = 0; i_row < 3; ++i_row) {
            for (int j_col = 0; j_col < 3; ++j_col) {
                EXPECT_NEAR(expected_R_ENU_ECEF[i_row][j_col],
                            T_ECEF_ENU[j_col][i_row],
                            rotation_check_threshold);
            }
        }

        // Check translation: For ECEF from ENU the norm is used
        double R = sqrt(pow(T_ECEF_ENU[0][3], 2) + pow(T_ECEF_ENU[1][3], 2) +
                        pow(T_ECEF_ENU[2][3], 2));
        EXPECT_NEAR(earth_radius_approx, R, 21e3);

        // Check last row
        EXPECT_NEAR(0.0, T_ECEF_ENU[3][0], 1e-6);
        EXPECT_NEAR(0.0, T_ECEF_ENU[3][1], 1e-6);
        EXPECT_NEAR(0.0, T_ECEF_ENU[3][2], 1e-6);
        EXPECT_NEAR(1.0, T_ECEF_ENU[3][3], 1e-6);
    }

    void checkT_ENU_ECEF(double T_ENU_ECEF[4][4],
                         double expected_R_ENU_ECEF[3][3]) {
        // Compare R's directly
        for (int i_row = 0; i_row < 3; ++i_row) {
            for (int j_col = 0; j_col < 3; ++j_col) {
                EXPECT_NEAR(expected_R_ENU_ECEF[i_row][j_col],
                            T_ENU_ECEF[i_row][j_col],
                            rotation_check_threshold);
            }
        }

        // Check translation: E == 0, N == 0, U == -earth radius
        EXPECT_NEAR(0.0, T_ENU_ECEF[0][3], 1e3);
        EXPECT_NEAR(0.0, T_ENU_ECEF[1][3], 1e3);
        EXPECT_NEAR(-earth_radius_approx, T_ENU_ECEF[2][3], 21e3);

        // Check last row
        EXPECT_NEAR(0.0, T_ENU_ECEF[3][0], 1e-6);
        EXPECT_NEAR(0.0, T_ENU_ECEF[3][1], 1e-6);
        EXPECT_NEAR(0.0, T_ENU_ECEF[3][2], 1e-6);
        EXPECT_NEAR(1.0, T_ENU_ECEF[3][3], 1e-6);
    }

    void checkTransformInverse(double T1[4][4], double T2[4][4]) {
        // Multiplication should produce identity
        for (int i_row = 0; i_row < 3; ++i_row) {
            for (int j_col = 0; j_col < 3; ++j_col) {
                double result_i_j = T1[i_row][0] * T2[0][j_col] +
                                    T1[i_row][1] * T2[1][j_col] +
                                    T1[i_row][2] * T2[2][j_col];
                if (i_row == j_col) {
                    EXPECT_NEAR(1.0, result_i_j, 1e-6);
                } else {
                    EXPECT_NEAR(0.0, result_i_j, 1e-6);
                }
            }
        }
    }

    void checkLLHDatumPipeline(double datum_llh[3],
                               double expected_R_ENU_ECEF[3][3]) {
        double T_ENU_ECEF[4][4];
        double T_ECEF_ENU[4][4];

        ecefFromENUTransformMatrix(datum_llh, T_ECEF_ENU);
        enuFromECEFTransformMatrix(datum_llh, T_ENU_ECEF);

        checkT_ENU_ECEF(T_ENU_ECEF, expected_R_ENU_ECEF);
        checkT_ECEF_ENU(T_ECEF_ENU, expected_R_ENU_ECEF);

        checkTransformInverse(T_ENU_ECEF, T_ECEF_ENU);
        checkTransformInverse(T_ECEF_ENU, T_ENU_ECEF);
    }

    void checkECEFDatumPipeline(double datum_ECEF[3],
                                double expected_R_ENU_ECEF[3][3]) {
        double T_ENU_ECEF[4][4];
        double T_ECEF_ENU[4][4];

        ecefFromENUTransformMatrix(datum_ECEF, T_ECEF_ENU, false);
        enuFromECEFTransformMatrix(datum_ECEF, T_ENU_ECEF, false);

        checkT_ENU_ECEF(T_ENU_ECEF, expected_R_ENU_ECEF);
        checkT_ECEF_ENU(T_ECEF_ENU, expected_R_ENU_ECEF);

        checkTransformInverse(T_ENU_ECEF, T_ECEF_ENU);
        checkTransformInverse(T_ECEF_ENU, T_ENU_ECEF);
    }

    void checkResult(double datum_llh[3], double expected_R_ENU_ECEF[3][3]) {
        // Check LLH datum version of pipeline
        checkLLHDatumPipeline(datum_llh, expected_R_ENU_ECEF);

        // Get ECEF datum from LLH and check ECEF version of pipeline
        double datum_ecef[3];
        ecefPointFromLLH(datum_llh, datum_ecef);
        checkECEFDatumPipeline(datum_ecef, expected_R_ENU_ECEF);
    }
};

/*
For near 90 degrees latitude, 0 degrees longitude:

+X <-> -N
+Y <-> +E
+Z <-> +U

X = [0.0, -1.0, 0.0]^T
Y = [1.0, 0.0, 0.0]^T
Z = [0.0, 0.0, 1.0]^T

R = [X|Y|Z]
*/
TEST_F(ECEFtoENUTest, LatNear90LongAt0) {
    double datum_llh[3] = {near90, 0.0, 0.0};

    double R_ENU_ECEF_result[3][3] = {{0.0, 1.0, 0.0},   //
                                      {-1.0, 0.0, 0.0},  //
                                      {0.0, 0.0, 1.0}};
    checkResult(datum_llh, R_ENU_ECEF_result);
}

/*
For near 90 degrees latitude, near 180 degrees longitude:

+X <-> +N
+Y <-> -E
+Z <-> +U

R is near
X = [0.0, 1.0, 0.0]^T
Y = [-1.0, 0.0, 0.0]^T
Z = [0.0, 0.0, 1.0]^T

R = [X|Y|Z]
*/
TEST_F(ECEFtoENUTest, LatNear90LongNear180) {
    double datum_llh[3] = {near90, near180, 0.0};

    double R_ENU_ECEF_result[3][3] = {{0.0, -1.0, 0.0},  //
                                      {1.0, 0.0, 0.0},   //
                                      {0.0, 0.0, 1.0}};

    checkResult(datum_llh, R_ENU_ECEF_result);
}

/*
For at 0 degrees latitude, 90 degrees longitude:

+X <-> -E
+Y <-> +U
+Z <-> +N

R is near
X = [-1.0, 0.0, 0.0]^T
Y = [0.0, 0.0, 1.0]^T
Z = [0.0, 1.0, 0.0]^T

R = [X|Y|Z]
*/
TEST_F(ECEFtoENUTest, LatNear0LongNear90) {
    double datum_llh[3] = {0.0, 90.0, 0.0};

    double R_ENU_ECEF_result[3][3] = {{-1.0, 0.0, 0.0},  //
                                      {0.0, 0.0, 1.0},   //
                                      {0.0, 1.0, 0.0}};


    checkResult(datum_llh, R_ENU_ECEF_result);
}

/*
For at 0 degrees latitude, -90 degrees longitude:

+X <-> +E
+Y <-> -U
+Z <-> +N

R is near
X = [1.0, 0.0, 0.0]^T
Y = [0.0, 0.0, -1.0]^T
Z = [0.0, 1.0, 0.0]^T

R = [X|Y|Z]
*/
TEST_F(ECEFtoENUTest, LatNear0LongNearN90) {
    double datum_llh[3] = {0.0, -90.0, 0.0};

    double R_ENU_ECEF_result[3][3] = {{1.0, 0.0, 0.0},  //
                                      {0.0, 0.0, 1.0},  //
                                      {0.0, -1.0, 0.0}};

    checkResult(datum_llh, R_ENU_ECEF_result);
}

/*
For near 90 degrees latitude, -90 degrees longitude:

+X <-> +E
+Y <-> +N
+Z <-> +U

R is near
X = [1.0, 0.0, 0.0]^T
Y = [0.0, 1.0, 0.0]^T
Z = [0.0, 0.0, 1.0]^T

R = [X|Y|Z]
*/
TEST_F(ECEFtoENUTest, IdentityPoint) {
    double datum_llh[3] = {near90, -90.0, 0.0};

    double R_ENU_ECEF_result[3][3] = {{1.0, 0.0, 0.0},  //
                                      {0.0, 1.0, 0.0},  //
                                      {0.0, 0.0, 1.0}};

    checkResult(datum_llh, R_ENU_ECEF_result);
}

}  // namespace wave
