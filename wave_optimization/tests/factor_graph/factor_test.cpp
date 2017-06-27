#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/Factor.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "example_instances.hpp"

namespace wave {

TEST(FactorTest, evaluateRaw) {
    auto meas = 1.23;

    using Pose2DVar = FactorVariable<Pose2D>;
    using Landmark2DVar = FactorVariable<Position2D>;


    DistanceToLandmarkFactor f{DistanceMeasurement{meas, 1.0},
                               std::make_shared<Pose2DVar>(),
                               std::make_shared<Landmark2DVar>()};

    // Prepare sample C-style arrays as used by Ceres
    const double param_pose_position[2] = {1.1, 2.2};
    const double param_pose_orientation[1] = {3.3};
    const double param_landmark[2] = {4.4, 5.5};
    double out_residuals[1];

    // Calculate expected values (use analytic jacobians)
    auto dist =
      std::sqrt((1.1 - 4.4) * (1.1 - 4.4) + (2.2 - 5.5) * (2.2 - 5.5));
    auto expected_residual = dist - meas;
    Vec3 expected_jac_pose{(1.1 - 4.4) / dist, (2.2 - 5.5) / dist, 0};
    Vec2 expected_jac_landmark{(1.1 - 4.4) / dist, (2.2 - 5.5) / dist};

    //    // Call and compare
    //    auto res = f.evaluateRaw(param_pose_position,
    //                             param_pose_orientation,
    //                             param_landmark,
    //                             out_residuals);
    //    EXPECT_TRUE(res);
    //    EXPECT_DOUBLE_EQ(expected_residual, out_residuals[0]);
}

TEST(FactorTest, constructPerfectPrior) {
    // Test that using a perfect prior immediately sets the variable's value
    // @todo this may change

    // Note explicitly constructing PerfectPrior is not intended for users -
    // They should use FactorGraph::addPerfectPrior. That is why this test does
    // some non-intuitive preparation (e.g. constructing a FactorMeasurement)
    using MeasType = FactorMeasurement<Distance, void>;
    using VarType = FactorVariable<Distance>;
    auto v = std::make_shared<VarType>();

    auto factor = PerfectPrior<Distance>{MeasType{1.2}, v};
    EXPECT_DOUBLE_EQ(1.2, v->value.value());

    EXPECT_TRUE(factor.isPerfectPrior());
}

TEST(FactorTest, print) {
    auto v1 = std::make_shared<Pose2DVar>();
    auto v2 = std::make_shared<Landmark2DVar>();
    auto meas = DistanceMeasurement{0.0, 1.0};

    auto factor = DistanceToLandmarkFactor{meas, v1, v2};

    std::stringstream expected;
    expected << "[Factor arity 2, variables: ";
    expected << *v1 << "(" << v1 << "), " << *v2 << "(" << v2 << ")]";

    std::stringstream ss;
    ss << factor;
    EXPECT_EQ(expected.str(), ss.str());
}

TEST(FactorTest, printPerfectPrior) {
    using MeasType = FactorMeasurement<Distance, void>;
    using VarType = FactorVariable<Distance>;
    auto v = std::make_shared<VarType>();
    auto factor = PerfectPrior<Distance>{MeasType{1.2}, v};

    std::stringstream expected;
    expected << "[Perfect prior for variable ";
    expected << *v << "(" << v << ")]";

    std::stringstream ss;
    ss << factor;
    EXPECT_EQ(expected.str(), ss.str());
}

}  // namespace wave
