#include <gtest/gtest.h>
#include "wave/geography/world_frame_conversions.hpp"

namespace wave {

class enuAndLLHPointConversionTest : public ::testing::Test {
 protected:
    double near180 = 180.0 - 1e-6;
    double cartesian_check_threshold = 1e-3;
    double llh_check_threshold = 1e-6;
    bool datum_is_llh;

    double square_size, diagonal_size;

    // Run tests based on a square of points in E-N plane:
    //
    //      A -- B
    //      |    |
    //      C -- D
    //
    // For either option of using LLH-define datums or ECEF defined datums:
    //
    // 1) Construct a virtual square of size S given a specified A datum point
    // 2) Compute the corresponding LLH ground truth points
    // 3) Set the datum points to be used based on ECEF datums or LLH datums
    // 4) For each point Q in the square
    //      Determine the ENU points of the other points with Q as the datum
    //      Confirm rough orientations
    //      Confirm point norms
    //      Confirm that conversion back to LLH matches
    //
    // with side length S, and datum point given as A as an input
    //
    // NOTE: Assumes test cases aren't at the poles where the ENU singularity
    // breaks the expected resulting orientations/locations for the square

    Eigen::Vector3d datum_A, datum_B, datum_C, datum_D;

    Eigen::Vector3d llh_A, llh_B, llh_C, llh_D;
    Eigen::Vector3d enu_B_wrt_A, enu_C_wrt_A, enu_D_wrt_A;
    Eigen::Vector3d enu_A_wrt_B, enu_C_wrt_B, enu_D_wrt_B;
    Eigen::Vector3d enu_A_wrt_C, enu_B_wrt_C, enu_D_wrt_C;
    Eigen::Vector3d enu_A_wrt_D, enu_B_wrt_D, enu_C_wrt_D;

    double calculateNormDiff(const Eigen::Vector3d input,
                             const Eigen::Vector3d expected) {
        Eigen::Vector3d diff = input - expected;
        return diff.norm();
    }

    /*double calculateNorm(const double input[3]) {
        double result =
          sqrt(pow(input[0], 2) + pow(input[1], 2) + pow(input[2], 2));
        return result;
    }*/

    // Set ground truth points as a square from ENU set at point A
    void problemSetup() {
        diagonal_size = sqrt(2.0) * square_size;

        enu_B_wrt_A << square_size, 0, 0;
        enu_C_wrt_A << 0, -square_size, 0;
        enu_D_wrt_A << square_size, -square_size, 0;

        /* enu_B_wrt_A(0) = square_size;
         enu_B_wrt_A(1) = 0;
         enu_B_wrt_A(2) = 0;

         enu_C_wrt_A(0) = 0;
         enu_C_wrt_A(1) = -square_size;
         enu_C_wrt_A(2) = 0;

         enu_D_wrt_A(0) = square_size;
         enu_D_wrt_A(1) = -square_size;
         enu_D_wrt_A(2) = 0;*/

        // Compute other LLH values from datum_A
        llhPointFromENU(enu_B_wrt_A, datum_A, llh_B, datum_is_llh);
        llhPointFromENU(enu_C_wrt_A, datum_A, llh_C, datum_is_llh);
        llhPointFromENU(enu_D_wrt_A, datum_A, llh_D, datum_is_llh);

        // Set datum points for other 3 points, and set llh_A
        if (datum_is_llh) {
            llh_A = datum_A;
            datum_B = llh_B;
            datum_C = llh_C;
            datum_D = llh_D;
        } else {
            llhPointFromECEF(datum_A, llh_A);
            ecefPointFromLLH(llh_B, datum_B);
            ecefPointFromLLH(llh_C, datum_C);
            ecefPointFromLLH(llh_D, datum_D);
        }
    }

    void checkHeights() {
        // Curvature of earth should increasingly raise heights out from datum
        EXPECT_LT(llh_A[2], llh_B[2]);
        EXPECT_LT(llh_A[2], llh_C[2]);
        EXPECT_LT(llh_B[2], llh_D[2]);
        EXPECT_LT(llh_C[2], llh_D[2]);
    }

    void checkENUFromA() {
        Eigen::Vector3d result_enu_B_from_A, result_enu_C_from_A,
          result_enu_D_from_A;

        enuPointFromLLH(llh_B, datum_A, result_enu_B_from_A, datum_is_llh);
        enuPointFromLLH(llh_C, datum_A, result_enu_C_from_A, datum_is_llh);
        enuPointFromLLH(llh_D, datum_A, result_enu_D_from_A, datum_is_llh);

        EXPECT_NEAR(0.0,
                    calculateNormDiff(enu_B_wrt_A, result_enu_B_from_A),
                    cartesian_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(enu_C_wrt_A, result_enu_C_from_A),
                    cartesian_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(enu_D_wrt_A, result_enu_D_from_A),
                    cartesian_check_threshold);

        // Check that mapping back to LLH matches original 4 points
        Eigen::Vector3d result_llh_B_from_A, result_llh_C_from_A,
          result_llh_D_from_A;
        llhPointFromENU(
          result_enu_B_from_A, datum_A, result_llh_B_from_A, datum_is_llh);
        llhPointFromENU(
          result_enu_C_from_A, datum_A, result_llh_C_from_A, datum_is_llh);
        llhPointFromENU(
          result_enu_D_from_A, datum_A, result_llh_D_from_A, datum_is_llh);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_B, result_llh_B_from_A),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_C, result_llh_C_from_A),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_D, result_llh_D_from_A),
                    llh_check_threshold);
    }

    void checkENUFromB() {
        Eigen::Vector3d result_enu_A_from_B, result_enu_C_from_B,
          result_enu_D_from_B;

        enuPointFromLLH(llh_A, datum_B, result_enu_A_from_B, datum_is_llh);
        enuPointFromLLH(llh_C, datum_B, result_enu_C_from_B, datum_is_llh);
        enuPointFromLLH(llh_D, datum_B, result_enu_D_from_B, datum_is_llh);

        EXPECT_NEAR(
          square_size, result_enu_A_from_B.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          diagonal_size, result_enu_C_from_B.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          square_size, result_enu_D_from_B.norm(), cartesian_check_threshold);

        // Check ENU signs
        EXPECT_TRUE(result_enu_A_from_B[0] < 0);
        EXPECT_TRUE(result_enu_C_from_B[0] < 0);
        EXPECT_TRUE(result_enu_C_from_B[1] < 0);
        EXPECT_TRUE(result_enu_D_from_B[1] < 0);

        // Check that mapping back to LLH matches original 4 points
        Eigen::Vector3d result_llh_A_from_B, result_llh_C_from_B,
          result_llh_D_from_B;
        llhPointFromENU(
          result_enu_A_from_B, datum_B, result_llh_A_from_B, datum_is_llh);
        llhPointFromENU(
          result_enu_C_from_B, datum_B, result_llh_C_from_B, datum_is_llh);
        llhPointFromENU(
          result_enu_D_from_B, datum_B, result_llh_D_from_B, datum_is_llh);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_A, result_llh_A_from_B),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_C, result_llh_C_from_B),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_D, result_llh_D_from_B),
                    llh_check_threshold);
    }

    void checkENUFromC() {
        Eigen::Vector3d result_enu_A_from_C, result_enu_B_from_C,
          result_enu_D_from_C;

        enuPointFromLLH(llh_A, datum_C, result_enu_A_from_C, datum_is_llh);
        enuPointFromLLH(llh_B, datum_C, result_enu_B_from_C, datum_is_llh);
        enuPointFromLLH(llh_D, datum_C, result_enu_D_from_C, datum_is_llh);

        EXPECT_NEAR(
          square_size, result_enu_A_from_C.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          diagonal_size, result_enu_B_from_C.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          square_size, result_enu_D_from_C.norm(), cartesian_check_threshold);

        // Check ENU signs
        EXPECT_TRUE(result_enu_A_from_C[1] > 0);
        EXPECT_TRUE(result_enu_B_from_C[0] > 0);
        EXPECT_TRUE(result_enu_B_from_C[1] > 0);
        EXPECT_TRUE(result_enu_D_from_C[0] > 0);

        // Check that mapping back to LLH matches original 4 points
        Eigen::Vector3d result_llh_A_from_C, result_llh_B_from_C,
          result_llh_D_from_C;
        llhPointFromENU(
          result_enu_A_from_C, datum_C, result_llh_A_from_C, datum_is_llh);
        llhPointFromENU(
          result_enu_B_from_C, datum_C, result_llh_B_from_C, datum_is_llh);
        llhPointFromENU(
          result_enu_D_from_C, datum_C, result_llh_D_from_C, datum_is_llh);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_A, result_llh_A_from_C),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_B, result_llh_B_from_C),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_D, result_llh_D_from_C),
                    llh_check_threshold);
    }

    void checkENUFromD() {
        Eigen::Vector3d result_enu_A_from_D, result_enu_B_from_D,
          result_enu_C_from_D;

        enuPointFromLLH(llh_A, datum_D, result_enu_A_from_D, datum_is_llh);
        enuPointFromLLH(llh_B, datum_D, result_enu_B_from_D, datum_is_llh);
        enuPointFromLLH(llh_C, datum_D, result_enu_C_from_D, datum_is_llh);

        EXPECT_NEAR(
          diagonal_size, result_enu_A_from_D.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          square_size, result_enu_B_from_D.norm(), cartesian_check_threshold);
        EXPECT_NEAR(
          square_size, result_enu_C_from_D.norm(), cartesian_check_threshold);

        // Check ENU signs
        EXPECT_TRUE(result_enu_A_from_D[0] < 0);
        EXPECT_TRUE(result_enu_A_from_D[1] > 0);
        EXPECT_TRUE(result_enu_B_from_D[1] > 0);
        EXPECT_TRUE(result_enu_C_from_D[0] < 0);

        // Check that mapping back to LLH matches original 4 points
        Eigen::Vector3d result_llh_A_from_D, result_llh_B_from_D,
          result_llh_C_from_D;
        llhPointFromENU(
          result_enu_A_from_D, datum_D, result_llh_A_from_D, datum_is_llh);
        llhPointFromENU(
          result_enu_B_from_D, datum_D, result_llh_B_from_D, datum_is_llh);
        llhPointFromENU(
          result_enu_C_from_D, datum_D, result_llh_C_from_D, datum_is_llh);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_A, result_llh_A_from_D),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_B, result_llh_B_from_D),
                    llh_check_threshold);
        EXPECT_NEAR(0.0,
                    calculateNormDiff(llh_C, result_llh_C_from_D),
                    llh_check_threshold);
    }

    void checkPipeline() {
        problemSetup();
        checkHeights();

        // Check the ENU points from all 4 datum choices to verify approximate
        // square
        checkENUFromA();
        checkENUFromB();
        checkENUFromC();
        checkENUFromD();
    }

    void checkResults(const Eigen::Vector3d datum_llh,
                      double input_square_size) {
        square_size = input_square_size;

        // Check LLH Pipeline
        datum_is_llh = true;
        datum_A = datum_llh;
        checkPipeline();

        // Check ECEF datum pipeline
        datum_is_llh = false;
        ecefPointFromLLH(datum_llh, datum_A);
        checkPipeline();
    }
};

TEST_F(enuAndLLHPointConversionTest, LatAt45LongAt0) {
    Eigen::Vector3d datum_llh(45, 0.0, 0.0);

    checkResults(datum_llh, 1.0);
    checkResults(datum_llh, 10.0);
    checkResults(datum_llh, 100.0);
    checkResults(datum_llh, 1000.0);
    checkResults(datum_llh, 10000.0);
}

TEST_F(enuAndLLHPointConversionTest, LatAt0LongUnder180Pos) {
    Eigen::Vector3d datum_llh(0, near180, 0.0);

    checkResults(datum_llh, 1.0);
    checkResults(datum_llh, 10.0);
    checkResults(datum_llh, 100.0);
    checkResults(datum_llh, 1000.0);
    checkResults(datum_llh, 10000.0);
}

TEST_F(enuAndLLHPointConversionTest, LatAt0LongUnder180Neg) {
    Eigen::Vector3d datum_llh(0, -near180, 0.0);

    checkResults(datum_llh, 1.0);
    checkResults(datum_llh, 10.0);
    checkResults(datum_llh, 100.0);
    checkResults(datum_llh, 1000.0);
    checkResults(datum_llh, 10000.0);
}

}  // namespace wave
