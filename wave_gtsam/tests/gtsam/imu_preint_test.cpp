#include <gtsam/navigation/CombinedImuFactor.h>

#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <wave/gtsam/preint_imu_factor.hpp>

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


static const Bias bias(Vector3(0.2, 0, 0),
                       Vector3(0, 0, 0.3));  // Biases (acc, rot)
static const Bias bias2(Vector3(0.2, 0.2, 0),
                        Vector3(1, 0, 0.3));  // Biases (acc, rot)

static const Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)),
                      Point3(5.0, 1.0, -50.0));
static const Vector3 v1(Vector3(0.5, 0.0, 0.0));
static const NavState state1(x1, v1);

Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
         Point3(5.5, 1.0, -50.0));
Vector3 v2(0.5, 0.0, 0.0);
static const NavState state2(x2, v2);

// Measurements
static const Vector3 measuredOmega(0, 0, M_PI / 10.0 + 0.3);
static const Vector3 measuredAcc =
  x1.rotation().unrotate(-kGravityAlongNavZDown) + Vector3(0.2, 0.0, 0.0);
static const double deltaT = 1.0;
static const double tol = 1e-6;

// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegratedCombinedMeasurements::Params> Params() {
    auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
    p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
    p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
    p->integrationCovariance = 0.0001 * I_3x3;
    return p;
}
}  // namespace common

/* ************************************************************************* */

// Test from gtsam, no libwave stuff
TEST(ImuFactor, ErrorAndJacobians) {
    using namespace common;
    PreintegratedImuMeasurements pim(
      common::Params(), Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    EXPECT_TRUE(assert_equal(state2, pim.predict(state1, bias)));

    PreintegratedCombinedMeasurements combined_pim(
      common::Params(), Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

    combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);


    // Create expected factor
    ImuFactor imuFactor(X(1), V(1), X(2), V(2), B(1), pim);

    noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(combined_pim.preintMeasCov());
    CombinedImuFactor combinedfactor(
      X(1), V(1), X(2), V(2), B(1), B(2), combined_pim);

    Vector errorExpected = imuFactor.evaluateError(x1, v1, x2, v2, bias);
    Vector errorActual =
      combinedfactor.evaluateError(x1, v1, x2, v2, bias, bias2);
    EXPECT_TRUE(assert_equal(errorExpected, errorActual.head(9), tol));

    // Expected Jacobians
    Matrix H1e, H2e, H3e, H4e, H5e;
    (void) imuFactor.evaluateError(
      x1, v1, x2, v2, bias, H1e, H2e, H3e, H4e, H5e);

    // Actual Jacobians
    Matrix H1a, H2a, H3a, H4a, H5a, H6a;
    (void) combinedfactor.evaluateError(
      x1, v1, x2, v2, bias, bias2, H1a, H2a, H3a, H4a, H5a, H6a);

    EXPECT_TRUE(assert_equal(H1e, H1a.topRows(9)));
    EXPECT_TRUE(assert_equal(H2e, H2a.topRows(9)));
    EXPECT_TRUE(assert_equal(H3e, H3a.topRows(9)));
    EXPECT_TRUE(assert_equal(H4e, H4a.topRows(9)));
    EXPECT_TRUE(assert_equal(H5e, H5a.topRows(9)));

    Values values;
    values.insert(X(1), x1);
    values.insert(V(1), v1);
    values.insert(X(2), x2);
    values.insert(V(2), v2);
    values.insert(B(1), bias);
    values.insert(B(2), bias2);
    EXPECT_TRUE(assert_equal(errorExpected, imuFactor.unwhitenedError(values)));

    // Make sure linearization is correct
    double diffDelta = 1e-7;
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactor", imuFactor, values, diffDelta, 1e-3));
}

// Test the factor wrapped in libwave stuff
TEST(WaveImuFactor, WaveErrorAndJacobians) {
    using namespace common;
    PreintegratedCombinedMeasurements combined_pim(
      common::Params(), Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

    combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

    // Create states
    auto s1 = wave::PoseVelBias{};
    s1.pose = x1;
    s1.vel << kZero, v1;

    auto s2 = wave::PoseVelBias{};
    s2.pose = x2;
    s2.vel << kZero, v2;

    // Create expected factor
    PreintegratedImuMeasurements pim(
      common::Params(), Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    ImuFactor imuFactor(X(1), V(1), X(2), V(2), B(1), pim);
    Vector errorExpected = imuFactor.evaluateError(x1, v1, x2, v2, bias);

    // Create our factor
    PreintegratedImuFactor<wave::PoseVelBias> factor{
      S(1), S(2), B(1), B(2), combined_pim};

    // Call evaluateError directly
    EXPECT_TRUE(assert_equal(
      errorExpected, factor.evaluateError(s1, s2, bias, bias2).head(9)));

    // Call with values
    Values values;
    values.insert(S(1), s1);
    values.insert(S(2), s2);
    values.insert(B(1), bias);
    values.insert(B(2), bias2);
    EXPECT_TRUE(
      assert_equal(errorExpected, factor.unwhitenedError(values).head(9)));

    // Make sure linearization is correct
    double diffDelta = 1e-7;
    EXPECT_TRUE(gtsam::internal::testFactorJacobians(
      "ImuFactor", factor, values, diffDelta, 1e-3));
}

}  // namespace wave
