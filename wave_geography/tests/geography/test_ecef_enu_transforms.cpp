#include <gtest/gtest.h>
#include "wave/geography/world_frame_conversions.hpp"
#include "wave/wave_test.hpp"

namespace wave {

class ECEFtoENUTest : public ::testing::Test {
 protected:
    double near90 = 90.0 - 1e-6;
    double near180 = 180.0 - 1e-6;
    double rotation_check_threshold = 1e-6;
    double earth_radius_approx = 6.371e6;  // +/- 21km between WGS84 max/min

    void checkT_ECEF_ENU(Eigen::Matrix4d T_ECEF_ENU,
                         Eigen::Matrix3d expected_R_ENU_ECEF) {
        // Compare R vs R^T
        EXPECT_PRED3(MatricesNearPrec,
                     expected_R_ENU_ECEF,
                     T_ECEF_ENU.topLeftCorner(3, 3).transpose(),
                     rotation_check_threshold);

        // Check translation: For ECEF from ENU the norm is used
        double R = sqrt(pow(T_ECEF_ENU(0, 3), 2) + pow(T_ECEF_ENU(1, 3), 2) +
                        pow(T_ECEF_ENU(2, 3), 2));
        EXPECT_NEAR(earth_radius_approx, R, 21e3);

        // Check last row
        EXPECT_NEAR(0.0, T_ECEF_ENU(3, 0), 1e-6);
        EXPECT_NEAR(0.0, T_ECEF_ENU(3, 1), 1e-6);
        EXPECT_NEAR(0.0, T_ECEF_ENU(3, 2), 1e-6);
        EXPECT_NEAR(1.0, T_ECEF_ENU(3, 3), 1e-6);
    }

    void checkT_ENU_ECEF(Eigen::Matrix4d T_ENU_ECEF,
                         Eigen::Matrix3d expected_R_ENU_ECEF) {
        // Compare R's directly
        EXPECT_PRED3(MatricesNearPrec,
                     expected_R_ENU_ECEF,
                     T_ENU_ECEF.topLeftCorner(3, 3),
                     rotation_check_threshold);

        // Check translation: E == 0, N == 0, U == -earth radius
        EXPECT_NEAR(0.0, T_ENU_ECEF(0, 3), 1e3);
        EXPECT_NEAR(0.0, T_ENU_ECEF(1, 3), 1e3);
        EXPECT_NEAR(-earth_radius_approx, T_ENU_ECEF(2, 3), 21e3);

        // Check last row
        EXPECT_NEAR(0.0, T_ENU_ECEF(3, 0), 1e-6);
        EXPECT_NEAR(0.0, T_ENU_ECEF(3, 1), 1e-6);
        EXPECT_NEAR(0.0, T_ENU_ECEF(3, 2), 1e-6);
        EXPECT_NEAR(1.0, T_ENU_ECEF(3, 3), 1e-6);
    }

    void checkTransformInverse(Eigen::Matrix4d T1, Eigen::Matrix4d T2) {
        // Multiplication should produce identity
        EXPECT_PRED3(
          MatricesNearPrec, T1 * T2, Eigen::Matrix4d::Identity(), 1e-6);
    }

    void checkLLHDatumPipeline(Eigen::Vector3d datum_llh,
                               Eigen::Matrix3d expected_R_ENU_ECEF) {
        Eigen::Vector3d datum_ECEF = ecefPointFromLLH(datum_llh);
        Eigen::Matrix4d T_ENU_ECEF = enuEcefTransformFromEcef(datum_ECEF);
        Eigen::Matrix4d T_ECEF_ENU = ecefEnuTransformFromEcef(datum_ECEF);

        checkT_ENU_ECEF(T_ENU_ECEF, expected_R_ENU_ECEF);
        checkT_ECEF_ENU(T_ECEF_ENU, expected_R_ENU_ECEF);

        checkTransformInverse(T_ENU_ECEF, T_ECEF_ENU);
        checkTransformInverse(T_ECEF_ENU, T_ENU_ECEF);
    }

    void checkECEFDatumPipeline(Eigen::Vector3d datum_ECEF,
                                Eigen::Matrix3d expected_R_ENU_ECEF) {
        Eigen::Matrix4d T_ENU_ECEF = enuEcefTransformFromEcef(datum_ECEF);
        Eigen::Matrix4d T_ECEF_ENU = ecefEnuTransformFromEcef(datum_ECEF);

        checkT_ENU_ECEF(T_ENU_ECEF, expected_R_ENU_ECEF);
        checkT_ECEF_ENU(T_ECEF_ENU, expected_R_ENU_ECEF);

        checkTransformInverse(T_ENU_ECEF, T_ECEF_ENU);
        checkTransformInverse(T_ECEF_ENU, T_ENU_ECEF);
    }

    void checkResult(Eigen::Vector3d datum_llh,
                     Eigen::Matrix3d expected_R_ENU_ECEF) {
        // Check LLH datum version of pipeline
        checkLLHDatumPipeline(datum_llh, expected_R_ENU_ECEF);

        // Get ECEF datum from LLH and check ECEF version of pipeline
        Eigen::Vector3d datum_ecef = ecefPointFromLLH(datum_llh);
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
    Eigen::Vector3d datum_llh(near90, 0.0, 0.0);

    Eigen::Matrix3d R_ENU_ECEF_result;
    R_ENU_ECEF_result << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
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
    // double datum_llh[3] = {near90, near180, 0.0};
    Eigen::Vector3d datum_llh(near90, near180, 0.0);

    Eigen::Matrix3d R_ENU_ECEF_result;
    R_ENU_ECEF_result << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

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
    Eigen::Vector3d datum_llh(0.0, 90.0, 0.0);

    Eigen::Matrix3d R_ENU_ECEF_result;
    R_ENU_ECEF_result << -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0;

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
    Eigen::Vector3d datum_llh(0.0, -90.0, 0.0);

    Eigen::Matrix3d R_ENU_ECEF_result;
    R_ENU_ECEF_result << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;

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
    Eigen::Vector3d datum_llh(near90, -90.0, 0.0);

    Eigen::Matrix3d R_ENU_ECEF_result;
    R_ENU_ECEF_result << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    checkResult(datum_llh, R_ENU_ECEF_result);
}

}  // namespace wave
