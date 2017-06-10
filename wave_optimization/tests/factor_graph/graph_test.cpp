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

    graph.addFactor<DistanceToLandmarkFactor>(2.3, p, l);

    ASSERT_EQ(1u, graph.countFactors());
}

TEST(FactorGraph, capacity) {
    FactorGraph graph;
    EXPECT_EQ(0u, graph.countFactors());
    EXPECT_TRUE(graph.empty());

    MatX m3 = MatX::Random(1, 3);

    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();
    graph.addFactor<DistanceToLandmarkFactor>(2.3, p, l);

    EXPECT_EQ(1u, graph.countFactors());
    EXPECT_FALSE(graph.empty());
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
    for (const auto &l : true_l_pos) {
        landmark_vars.push_back(std::make_shared<Landmark2DVar>(l));
        // For now, landmark positions are exactly known.
        landmark_vars.back()->setFixed(true);
    }
    for (const auto &pose : true_poses) {
        // Make uninitialized Variable
        auto p = std::make_shared<Pose2DVar>();
        pose_vars.push_back(p);
        for (auto i = 0u; i < landmark_vars.size(); ++i) {
            auto distance = double{(true_l_pos[i] - pose.head<2>()).norm()};
            graph.addFactor<DistanceToLandmarkFactor>(
              distance, p, landmark_vars[i]);
        }
    }

    EXPECT_EQ(true_poses.size() * true_l_pos.size(), graph.countFactors());

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

    graph.addFactor<DistanceToLandmarkFactor>(0.0, v1, l);
    graph.addFactor<DistanceToLandmarkFactor>(1.1, v2, l);
    graph.addFactor<DistanceToLandmarkFactor>(2.2, v3, l);

    // Currently factors are stored privately in graph, so make our own to
    // generate the expected string
    auto f1 = DistanceToLandmarkFactor{0.0, v1, l};
    auto f2 = DistanceToLandmarkFactor{1.1, v2, l};
    auto f3 = DistanceToLandmarkFactor{2.2, v3, l};

    std::stringstream expected;
    expected << "FactorGraph 3 factors [";
    expected << f1 << ", " << f2 << ", " << f3;
    expected << "]";

    std::stringstream ss;
    ss << graph;
    EXPECT_EQ(expected.str(), ss.str());
}

}  // namespace wave
