#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <wave/gtsam/preintg_imu_factor/preintg_imu_factor.hpp>

#include "wave/wave_test.hpp"

using namespace gtsam;

namespace wave {

// Common linearization point and measurements for tests
namespace common {
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::S;

static const Vector3 kZero = Z_3x1;
typedef imuBias::ConstantBias Bias;
static const Bias kZeroBiasHat, kZeroBias;

static const Vector3 kZeroOmegaCoriolis(0, 0, 0);
static const Vector3 kNonZeroOmegaCoriolis(0, 0.1, 0.1);

static const double kGravity = 10;
static const Vector3 kGravityAlongNavZDown(0, 0, kGravity);

// Realistic MEMS white noise characteristics. Angular and velocity random walk
// expressed in degrees respectively m/s per sqrt(hr).
auto radians = [](double t) { return t * M_PI / 180; };
static const double kGyroSigma = radians(0.5) / 60;  // 0.5 degree ARW
static const double kAccelSigma = 0.1 / 60;          // 10 cm VRW

static const Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
                      Point3(5.0, 1.0, 0));
static const Vector3 v1(Vector3(0.5, 0.0, 0.0));
static const NavState state1(x1, v1);

// Measurements
static const double w = M_PI / 100;
static const Vector3 measuredOmega(w, 0, 0);
static const Vector3 measuredAcc =
  x1.rotation().unrotate(-kGravityAlongNavZDown);
static const double deltaT = 1.0;

static const Pose3 x2(Rot3::RzRyRx(M_PI / 12.0 + w, M_PI / 6.0, M_PI / 4.0),
                      Point3(5.5, 1.0, 0));
static const Vector3 v2(Vector3(0.5, 0.0, 0.0));
static const NavState state2(x2, v2);

// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegrationParams> Params() {
    auto p = PreintegrationParams::MakeSharedD(kGravity);
    p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
    p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
    p->integrationCovariance = 0.0001 * I_3x3;
    return p;
}
}  // namespace common

/* ************************************************************************* */
namespace {
using namespace common;
// Auxiliary functions to test evaluate error in ImuFactor
/* ************************************************************************* */
Rot3 evaluateRotationError(const ImuFactor &factor,
                           const Pose3 &pose_i,
                           const Vector3 &vel_i,
                           const Pose3 &pose_j,
                           const Vector3 &vel_j,
                           const Bias &bias) {
    return Rot3::Expmap(
      factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias).head(3));
}

}  // namespace

// Test from gtsam, no libwave stuff
TEST(ImuFactor, ErrorAndJacobians) {
    using namespace common;
    PreintegratedImuMeasurements pim(common::Params());

    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    EXPECT_TRUE(assert_equal(state2, pim.predict(state1, kZeroBias)));

    // Create factor
    ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

    // Expected error
    Vector expectedError(9);
    expectedError << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    EXPECT_TRUE(assert_equal(expectedError,
                             factor.evaluateError(x1, v1, x2, v2, kZeroBias)));

    Values values;
    values.insert(X(1), x1);
    values.insert(V(1), v1);
    values.insert(X(2), x2);
    values.insert(V(2), v2);
    values.insert(B(1), kZeroBias);
    EXPECT_TRUE(assert_equal(expectedError, factor.unwhitenedError(values)));

    // Make sure linearization is correct
    double diffDelta = 1e-7;
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactor", factor, values, diffDelta, 1e-3));


    // Actual Jacobians
    Matrix H1a, H2a, H3a, H4a, H5a;
    (void) factor.evaluateError(
      x1, v1, x2, v2, kZeroBias, H1a, H2a, H3a, H4a, H5a);

    // Make sure rotation part is correct when error is interpreted as
    // axis-angle
    // Jacobians are around zero, so the rotation part is the same as:
    Matrix H1Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, kZeroBias),
      x1);
    EXPECT_TRUE(assert_equal(H1Rot3, H1a.topRows(3)));

    Matrix H3Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, kZeroBias),
      x2);
    EXPECT_TRUE(assert_equal(H3Rot3, H3a.topRows(3)));

    // Evaluate error with wrong values
    Vector3 v2_wrong = v2 + Vector3(0.1, 0.1, 0.1);
    values.update(V(2), v2_wrong);
    expectedError << 0, 0, 0, 0, 0, 0, -0.0724744871, -0.040715657,
      -0.151952901;
    EXPECT_TRUE(
      assert_equal(expectedError,
                   factor.evaluateError(x1, v1, x2, v2_wrong, kZeroBias),
                   1e-2));
    EXPECT_TRUE(
      assert_equal(expectedError, factor.unwhitenedError(values), 1e-2));

    // Make sure the whitening is done correctly
    Matrix cov = pim.preintMeasCov();
    Matrix R = RtR(cov.inverse());
    Vector whitened = R * expectedError;
    EXPECT_TRUE(
      assert_equal(0.5 * whitened.squaredNorm(), factor.error(values), 1e-4));

    // Make sure linearization is correct
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactorWrongValues", factor, values, diffDelta, 1e-3));
}


// Test the factor wrapped in libwave stuff
TEST(WaveImuFactor, WaveErrorAndJacobians) {
    using namespace common;
    PreintegratedImuMeasurements pim(common::Params());

    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    EXPECT_TRUE(assert_equal(state2, pim.predict(state1, kZeroBias)));

    // Create states
    auto s1 = wave::PoseVelBias{};
    s1.pose = x1;
    s1.vel << kZero, v1;

    auto s2 = wave::PoseVelBias{};
    s2.pose = x2;
    s2.vel << kZero, v2;

    // Create factor
    PreintgIMUFactor factor(S(1), S(2), B(1), pim);

    // Expected error
    Vector expectedError(9);
    expectedError.setZero();
    EXPECT_TRUE(
      assert_equal(expectedError, factor.evaluateError(s1, s2, kZeroBias)));


    Values values;
    values.insert(S(1), s1);
    values.insert(S(2), s2);
    values.insert(B(1), kZeroBias);

    EXPECT_TRUE(assert_equal(expectedError, factor.unwhitenedError(values)));

    // Make sure linearization is correct
    double diffDelta = 1e-7;
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactor", factor, values, diffDelta, 1e-3));


    // Evaluate error with wrong values
    Vector3 v2_wrong = v2 + Vector3(0.1, 0.1, 0.1);
    s2.vel.tail<3>() = v2_wrong;
    values.update(S(2), s2);
    expectedError.tail<3>() << -0.0724744871, -0.040715657, -0.151952901;
    EXPECT_TRUE(assert_equal(
      expectedError, factor.evaluateError(s1, s2, kZeroBias), 1e-2));

    EXPECT_TRUE(
      assert_equal(expectedError, factor.unwhitenedError(values), 1e-2));

    // Make sure the whitening is done correctly
    Matrix cov = pim.preintMeasCov();
    Matrix R = RtR(cov.inverse());
    Vector whitened = R * expectedError;
    EXPECT_TRUE(
      assert_equal(0.5 * whitened.squaredNorm(), factor.error(values), 1e-4));

    // Make sure linearization is correct
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactorWrongValues", factor, values, diffDelta, 1e-3));
}

}  // namespace wave
