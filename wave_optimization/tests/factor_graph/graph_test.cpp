#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/graph.hpp"

namespace wave {

TEST(FactorGraph, add) {
    // add unary factor
    FactorGraph graph1;
    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();

    graph1.addFactor<DistanceToLandmarkFactor>(2.3, p, l);

    ASSERT_EQ(2u, graph1.countFactors());
    ASSERT_EQ(2u, graph1.countVariables());
}

TEST(FactorGraph, capacity) {
    FactorGraph graph;
    EXPECT_EQ(0u, graph.size());
    EXPECT_EQ(0u, graph.countFactors());
    EXPECT_EQ(0u, graph.countVariables());
    EXPECT_TRUE(graph.empty());

    MatX m3 = MatX::Random(1, 3);

    auto p = std::make_shared<Pose2DVar>();
    auto l = std::make_shared<Landmark2DVar>();
    graph.addFactor<DistanceToLandmarkFactor>(2.3, p, l);

    EXPECT_EQ(3u, graph.size());
    EXPECT_EQ(1u, graph.countFactors());
    EXPECT_EQ(2u, graph.countVariables());
    EXPECT_FALSE(graph.empty());
}

TEST(FactorGraph, Sim) {
    // Simulate a robot passing by a landmark
    Vec2 landmark_pos;
    landmark_pos << 2, 1.8;
    Vec3 pose = Vec3::Zero();
    FactorGraph graph;
    auto l = std::make_shared<Landmark2DVar>();

    for (int i = 0; i < 5; ++i) {
        pose(0) += 1;
        pose(1) += 0.5;
        double distance = (landmark_pos - pose.head<2>()).norm();
        auto p = std::make_shared<Pose2DVar>();
        graph.addFactor<DistanceToLandmarkFactor>(distance, p, l);
    }
}

}  // end of wave namespace
