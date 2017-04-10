#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/factor_graph.hpp"


namespace wave {

TEST(Factor, constructor) {
    Factor f1(2, 100);
    ASSERT_EQ(1, f1.arity);
    ASSERT_EQ(2, f1.connections[0]);
    ASSERT_EQ(100, *static_cast<double *>(f1.measurement));

    Factor f2(1, 2, 100);
    ASSERT_EQ(2, f2.arity);
    ASSERT_EQ(1, f2.connections[0]);
    ASSERT_EQ(2, f2.connections[1]);
    ASSERT_EQ(100, *static_cast<double *>(f2.measurement));
}

TEST(FactorGraph, addUnaryFactor) {
    FactorGraph graph;
    graph.addUnaryFactor(1, 2);

    ASSERT_EQ(1, (int) graph.graph.size());
    ASSERT_EQ(1, (int) graph.factors.size());
}

TEST(FactorGraph, addBinaryFactor) {
    FactorGraph graph;

    graph.addBinaryFactor(1, 2, 2);
    ASSERT_EQ(2, (int) graph.graph.size());
    ASSERT_EQ(1, (int) graph.factors.size());
}

TEST(FactorGraph, print) {
    FactorGraph graph;

    graph.addUnaryFactor(1, 2);
    graph.addBinaryFactor(1, 2, 2);
    graph.addBinaryFactor(2, 3, 2);
    graph.print();
}

}  // end of wave namespace
