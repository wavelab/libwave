#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/Factor.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "example_instances.hpp"

namespace wave {

TEST(FactorTest, evaluate) {
    auto meas = 1.23;

    using Pose2DVar = FactorVariable<Pose2D>;
    using Landmark2DVar = FactorVariable<Position2D>;


    DistanceToLandmarkFactor f{DistanceMeasurement{meas, 1.0},
                               std::make_shared<Pose2DVar>(),
                               std::make_shared<Landmark2DVar>()};

    // Prepare test inputs
    auto param_pose =
      Pose2D<double>{Position2D<double>{1.1, 2.2}, Orientation2D<double>{3.3}};
    auto param_landmark = Position2D<double>{4.4, 5.5};
    auto residual = Distance<double>{};

    // Calculate expected values (use analytic jacobians)
    auto dist = std::hypot(1.1 - 4.4, 2.2 - 5.5);
    auto expected_residual = dist - meas;

    // Call and compare
    auto res = f.evaluate(param_pose, param_landmark, residual);
    EXPECT_TRUE(res);
    EXPECT_DOUBLE_EQ(expected_residual, residual.value());
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
