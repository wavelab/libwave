#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorGraph.hpp"
#include "example_instances.hpp"
#include "wave/optimization/ceres/graph_wrapper.hpp"

namespace wave {

TEST(FactorGraph, add) {
    // add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();
    auto m = DistanceMeasurement{2.3, 1.0};

    graph.addFactor(distanceMeasurementFunction, m, p, l);

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
    graph.addFactor(distanceMeasurementFunction, m, p, l);

    EXPECT_EQ(1u, graph.countFactors());
    EXPECT_FALSE(graph.empty());
}

TEST(FactorGraph, addPrior) {
    const auto test_meas = Vec2{1.2, 3.4};
    const auto test_stddev = Vec2{0.01, 0.01};
    const auto test_val = Vec2{1.23, 3.38};
    // The expected residuals are normalized
    const Vec2 expected_res = (test_val - test_meas).cwiseQuotient(test_stddev);

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Landmark2DVar>();
    auto m = FactorMeasurement<Landmark2D>{test_meas, test_stddev};

    // Prepare arguments in a form matching ceres calls
    Vec2 test_residual;
    Mat2 test_jac;
    const double *const params[] = {test_val.data()};
    double *jacs[] = {test_jac.data()};

    // Add the factor, retrieve the pointer to it, and verity
    graph.addPrior(m, p);
    EXPECT_EQ(1u, graph.countFactors());

    auto factor = *graph.begin();
    ASSERT_NE(nullptr, factor);

    EXPECT_TRUE(factor->evaluateRaw(params, test_residual.data(), jacs));
    EXPECT_PRED2(VectorsNear, expected_res, test_residual);
    EXPECT_PRED2(MatricesNear, Mat2::Identity(), test_jac);
}

TEST(FactorGraph, addPriorOfSize1) {
    const auto test_meas = 1.2;
    const auto test_stddev = 0.01;
    const auto test_val = 1.23;
    const auto expected_res = (test_val - test_meas) / test_stddev;

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<FactorVariable<double>>();
    auto m = FactorMeasurement<double>{test_meas, test_stddev};

    // Prepare arguments in a form matching ceres calls
    double test_residual;
    double test_jac;
    const double *const params[] = {&test_val};
    double *jacs[] = {&test_jac};

    // Add the factor, retrieve the pointer to it, and verity
    graph.addPrior(m, p);
    EXPECT_EQ(1u, graph.countFactors());

    auto factor = *graph.begin();
    ASSERT_NE(nullptr, factor);

    EXPECT_TRUE(factor->evaluateRaw(params, &test_residual, jacs));
    EXPECT_DOUBLE_EQ(expected_res, test_residual);
    EXPECT_DOUBLE_EQ(1.0, test_jac);
}

TEST(FactorGraph, addPerfectPrior) {
    const auto test_meas = Vec2{1.2, 3.4};
    const auto test_val = Vec2{1.23, 3.38};

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<Landmark2DVar>();

    // Prepare arguments in a form matching ceres calls
    Vec2 test_residual;
    Mat2 test_jac;
    const double *const params[] = {test_val.data()};
    double *jacs[] = {test_jac.data()};

    // Add the factor
    graph.addPerfectPrior(test_meas, p);
    EXPECT_EQ(1u, graph.countFactors());

    // The variable should have the measured value immediately
    // @todo this may change
    EXPECT_PRED2(VectorsNear, test_meas, Eigen::Map<Vec2>(p->data()));

    // Retrieve a pointer to the factor we just (indirectly) added
    auto factor = *graph.begin();
    ASSERT_NE(nullptr, factor);

    EXPECT_TRUE(factor->isPerfectPrior());

    // We cannot evaluate a factor that is a perfect prior
    EXPECT_FALSE(factor->evaluateRaw(params, test_residual.data(), jacs));
}

TEST(FactorGraph, addPerfectPriorOfSize1) {
    const auto test_meas = 1.2;
    const auto test_val = 1.23;

    // Prepare arguments to add unary factor
    FactorGraph graph;
    auto p = std::make_shared<FactorVariable<double>>();

    // Prepare arguments in a form matching ceres calls
    double test_residual;
    double test_jac;
    const double *const params[] = {&test_val};
    double *jacs[] = {&test_jac};

    // Add the factor
    graph.addPerfectPrior(test_meas, p);
    EXPECT_EQ(1u, graph.countFactors());

    // The variable should have the measured value immediately
    // @todo this may change
    EXPECT_DOUBLE_EQ(test_meas, *p->data());

    auto factor = *graph.begin();
    ASSERT_NE(nullptr, factor);

    EXPECT_TRUE(factor->isPerfectPrior());

    // We cannot evaluate a factor that is a perfect prior
    EXPECT_FALSE(factor->evaluateRaw(params, &test_residual, jacs));
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
            graph.addFactor(
              distanceMeasurementFunction, meas, p, landmark_vars[i]);
        }
    }

    // Expect one factor for each measurement, plus the three priors
    EXPECT_EQ(3 + true_poses.size() * true_l_pos.size(), graph.countFactors());

    // Fill the variables with values
    evaluateGraph(graph);

    // Make sure the poses were triangulated correctly
    for (auto i = 0u; i < true_l_pos.size(); ++i) {
        EXPECT_PRED2(VectorsNear,
                     true_l_pos[i],
                     Eigen::Map<Vec2>(landmark_vars[i]->value.data()));
    }
}

TEST(GraphTest, print) {
    auto l = std::make_shared<Landmark2DVar>();
    auto v1 = std::make_shared<Pose2DVar>();
    auto v2 = std::make_shared<Pose2DVar>();
    auto v3 = std::make_shared<Pose2DVar>();

    auto graph = FactorGraph{};

    auto m1 = DistanceMeasurement{0.0, 1.0};
    auto m2 = DistanceMeasurement{1.1, 1.0};
    auto m3 = DistanceMeasurement{2.2, 1.0};

    graph.addFactor(distanceMeasurementFunction, m1, v1, l);
    graph.addFactor(distanceMeasurementFunction, m2, v2, l);
    graph.addFactor(distanceMeasurementFunction, m3, v3, l);

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
