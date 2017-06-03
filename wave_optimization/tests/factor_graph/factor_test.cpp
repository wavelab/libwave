#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/Factor.hpp"

namespace wave {

TEST(FactorTest, evaluate) {
    auto meas = 1.23;
    DistanceToLandmarkFactor f{
      meas, std::make_shared<Pose2DVar>(), std::make_shared<Landmark2DVar>()};

    // Prepare sample C-style arrays as used by Ceres
    const double param_pose[3] = {1.1, 2.2, 3.3};
    const double param_landmark[2] = {4.4, 5.5};
    const double const *parameters[2] = {param_pose, param_landmark};
    double out_residuals[1];
    double out_jac_pose[3];
    double out_jac_landmark[2];
    double *out_jacobians[2] = {out_jac_pose, out_jac_landmark};

    // Calculate expected values (use analytic jacobians)
    auto dist =
      std::sqrt((1.1 - 4.4) * (1.1 - 4.4) + (2.2 - 5.5) * (2.2 - 5.5));
    auto expected_residual = dist - meas;
    Vec3 expected_jac_pose{(1.1 - 4.4) / dist, (2.2 - 5.5) / dist, 0};
    Vec2 expected_jac_landmark{(1.1 - 4.4) / dist, (2.2 - 5.5) / dist};

    // Call and compare
    auto res = f.evaluate(parameters, out_residuals, out_jacobians);
    EXPECT_TRUE(res);
    EXPECT_DOUBLE_EQ(expected_residual, out_residuals[0]);
    EXPECT_PRED2(
      VectorsNear, expected_jac_pose, Eigen::Map<Vec3>{out_jac_pose});
    EXPECT_PRED2(
      VectorsNear, expected_jac_landmark, Eigen::Map<Vec2>{out_jac_landmark});
}

TEST(FactorTest, print) {
    auto v1 = std::make_shared<Pose2DVar>();
    auto v2 = std::make_shared<Landmark2DVar>();

    DistanceToLandmarkFactor factor{0.0, v1, v2};

    std::stringstream expected;
    expected << "[Factor arity 2, variables: ";
    expected << *v1 << "(" << v1 << "), " << *v2 << "(" << v2 << ")]";

    std::stringstream ss;
    ss << factor;
    EXPECT_EQ(expected.str(), ss.str());
}

}  // namespace wave
