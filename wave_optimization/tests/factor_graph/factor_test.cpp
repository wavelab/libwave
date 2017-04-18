#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/factor.hpp"


namespace wave {

TEST(Factor, constructor) {
    // TEST UNARY FACTOR
    auto var = std::make_shared<Variable>(VariableType::POSE, 1);
    UnaryFactor<double> f1(var, 100.0);

    EXPECT_EQ(1u, f1.variables[0]->id);
    EXPECT_DOUBLE_EQ(100.0, f1.measurement);

    // TEST BINARY FACTOR
    auto var1 = std::make_shared<Variable>(VariableType::POSE, 1);
    auto var2 = std::make_shared<Variable>(VariableType::POSE, 2);
    BinaryFactor<double> f2(var1, var2, 100.0);

    EXPECT_EQ(1u, f2.variables[0]->id);
    EXPECT_EQ(2u, f2.variables[1]->id);
    EXPECT_DOUBLE_EQ(100.0, f2.measurement);
}

}  // end of wave namespace
