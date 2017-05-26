#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/graph.hpp"


namespace wave {

TEST(FactorGraph, add) {
    // add unary factor
    FactorGraph graph1;
    MatX m1 = MatX::Random(1, 3);
    MatX m2 = MatX::Random(1, 3);

    graph1.add<PoseVar>(1, m1);
    graph1.add<PoseVar>(2, m2);

    ASSERT_EQ(2u, graph1.factors.size());
    ASSERT_EQ(2u, graph1.variables.size());

    // add binary factor
    FactorGraph graph2;
    MatX m3 = MatX::Random(1, 3);

    graph2.add<PoseVar, PoseVar>(1, 2, m3);
    ASSERT_EQ(1u, graph2.factors.size());
    ASSERT_EQ(2u, graph2.variables.size());
}

TEST(FactorGraph, print) {
    FactorGraph graph;
    MatX m1 = MatX::Random(1, 3);
    MatX m2 = MatX::Random(1, 3);

    graph.add<PoseVar>(1, m1);
    graph.add<PoseVar>(2, m2);

    ASSERT_EQ(2u, graph.factors.size());
    std::cout << graph << std::endl;
}

}  // end of wave namespace
