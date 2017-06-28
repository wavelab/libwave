#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "example_instances.hpp"
#include "wave/optimization/ceres/CeresOptimizer.hpp"

namespace wave {

TEST(FactorGraph, add) {
    // add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();
    auto m = DistanceMeasurement{2.3, 1.0};

    graph.addFactor<DistanceMeasurementFunctor>(m, p, l);

    ASSERT_EQ(1u, graph.countFactors());
}

TEST(FactorGraph, capacity) {
    FactorGraph graph;
    EXPECT_EQ(0u, graph.countFactors());
    EXPECT_TRUE(graph.empty());

    MatX m3 = MatX::Random(1, 3);

    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();
    auto m = DistanceMeasurement{2.3, 1.0};
    graph.addFactor<DistanceMeasurementFunctor>(m, p, l);

    EXPECT_EQ(1u, graph.countFactors());
    EXPECT_FALSE(graph.empty());
}

TEST(FactorGraph, addPrior) {
    const auto test_meas = Vec2{1.2, 3.4};
    const auto test_stddev = Vec2{0.01, 0.01};
    //    const auto test_val = Vec2{1.23, 3.38};
    //    // The expected results are normalized
    //    const Vec2 expected_res = (test_val -
    //    test_meas).cwiseQuotient(test_stddev);
    //    const Mat2 expected_jac = Mat2::Identity() / 0.01;

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Landmark2DVar>();
    auto m = FactorMeasurement<Position2D>{test_meas, test_stddev};

    // Add the factor
    graph.addPrior(m, p);
    EXPECT_EQ(1u, graph.countFactors());

    // @todo? evaluate
}

TEST(FactorGraph, addPerfectPrior) {
    const auto test_meas = Position2D<double>{1.2, 3.4};
    //    const auto test_val = Position2D<double>{1.23, 3.38};

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Landmark2DVar>();

    // Add the factor
    graph.addPerfectPrior(test_meas, p);
    EXPECT_EQ(1u, graph.countFactors());

    // The variable should have the measured value immediately
    // @todo this may change
    EXPECT_PRED2(VectorsNear, test_meas, p->value);

    // Retrieve a pointer to the factor we just (indirectly) added
    auto factor = *graph.begin();
    ASSERT_NE(nullptr, factor);

    EXPECT_TRUE(factor->isPerfectPrior());

    // @todo? evaluate
}

TEST(FactorGraph, triangulationSim) {
    // Simulate a robot passing by a landmark
    // First generate "true" data
    auto true_l_pos = std::vector<Vec2>{{2.0, 1.8}, {3.0, 1.8}, {2.5, 0.7}};

    auto true_poses = std::vector<Vec3>{{0, 0, 0}};
    for (int i = 1; i < 5; ++i) {
        true_poses.push_back(true_poses.back() + Vec3{1., 0.5, 0});
    }

    // Construct variables and add them to the graph
    auto landmark_vars = std::vector<std::shared_ptr<Landmark2DVar>>{};
    auto pose_vars = std::vector<std::shared_ptr<Pose2DVar>>{};

    FactorGraph graph;

    for (const auto &l_pos : true_l_pos) {
        auto l = std::make_shared<Landmark2DVar>();
        landmark_vars.push_back(l);
        // For now, landmark positions are exactly known.
        graph.addPerfectPrior(l_pos, l);
    }

    for (const auto &pose : true_poses) {
        // Make uninitialized Variable
        auto p = std::make_shared<Pose2DVar>();
        pose_vars.push_back(p);
        for (auto i = 0u; i < landmark_vars.size(); ++i) {
            auto distance = double{(true_l_pos[i] - pose.head<2>()).norm()};
            auto meas = DistanceMeasurement{distance, 1.0};
            graph.addFactor<DistanceMeasurementFunctor>(
              meas, p, landmark_vars[i]);
        }
    }

    // Expect one factor for each measurement, plus the three priors
    EXPECT_EQ(3 + true_poses.size() * true_l_pos.size(), graph.countFactors());

    // Fill the variables with values
    graph.evaluate();

    // Make sure the poses were triangulated correctly
    for (auto i = 0u; i < true_l_pos.size(); ++i) {
        EXPECT_PRED2(VectorsNear,
                     true_l_pos[i],
                     Eigen::Map<Vec2>(landmark_vars[i]->value.data()));
    }
}

TEST(FactorGraph, print) {
    auto l = std::make_shared<Landmark2DVar>();
    auto v1 = std::make_shared<Pose2DVar>();
    auto v2 = std::make_shared<Pose2DVar>();
    auto v3 = std::make_shared<Pose2DVar>();

    auto &&graph = FactorGraph{};

    auto m1 = DistanceMeasurement{0.0, 1.0};
    auto m2 = DistanceMeasurement{1.1, 1.0};
    auto m3 = DistanceMeasurement{2.2, 1.0};

    graph.addFactor<DistanceMeasurementFunctor>(m1, v1, l);
    graph.addFactor<DistanceMeasurementFunctor>(m2, v2, l);
    graph.addFactor<DistanceMeasurementFunctor>(m3, v3, l);

    // Currently factors are stored privately in graph, so make our own to
    // generate the expected string
    auto f1 = DistanceToLandmarkFactor{m1, v1, l};
    auto f2 = DistanceToLandmarkFactor{m2, v2, l};
    auto f3 = DistanceToLandmarkFactor{m3, v3, l};

    std::stringstream expected;
    expected << "FactorGraph 3 factors [";
    expected << f1 << ", " << f2 << ", " << f3;
    expected << "]";

    std::stringstream ss;
    ss << graph;
    EXPECT_EQ(expected.str(), ss.str());
}

}  // namespace wave
