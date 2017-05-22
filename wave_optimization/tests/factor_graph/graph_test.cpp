#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/graph.hpp"


namespace wave {

TEST(FactorGraph, addUnaryFactor) {
    FactorGraph graph;
    graph.addUnaryFactor<PoseVar, double>(1, 2.0);
    graph.addUnaryFactor<PoseVar, int>(1, 1);

    ASSERT_EQ(2u, graph.graph.size());
    ASSERT_EQ(2u, graph.factors.size());
}

TEST(FactorGraph, addBinaryFactor) {
    FactorGraph graph;

    graph.addBinaryFactor<PoseVar, PoseVar, double>(1, 2, 2);
    ASSERT_EQ(2u, graph.graph.size());
    ASSERT_EQ(1u, graph.factors.size());
}

TEST(FactorGraph, print) {
    FactorGraph graph;

    graph.addUnaryFactor<PoseVar, int>(1, 1);
    graph.addBinaryFactor<PoseVar, PoseVar, int>(2, 3, 2);
    graph.addBinaryFactor<PoseVar, PoseVar, int>(3, 4, 3);

    Vec2 z;
    z << 100.0, 200.0;
    graph.addBinaryFactor<PoseVar, PoseVar, Vec2>(1, 2, z);
    graph.addBinaryFactor<PoseVar, PoseVar, Vec2>(2, 3, z);

    std::cout << graph << std::endl;
}

}  // end of wave namespace
