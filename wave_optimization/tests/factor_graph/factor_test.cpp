#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/factor.hpp"


namespace wave {

TEST(Factor, constructor) {
    MatX A(3, 3);
    MatX expected(3, 3);

    // TEST UNARY FACTOR
    // clang-format off
    A << 0.0, 1.0, 2.0,
         3.0, 4.0, 5.0,
         6.0, 7.0, 8.0;
    expected << 0.0, 1.0, 2.0,
                3.0, 4.0, 5.0,
                6.0, 7.0, 8.0;
    // clang-format on
    auto var = std::make_shared<FactorVariable>(1, 2);
    UnaryFactor f1(var, A);

    EXPECT_EQ(1u, f1.variables[0]->id);
    EXPECT_TRUE(f1.measurement.isApprox(expected));
    std::cout << f1 << std::endl;

    // TEST BINARY FACTOR
    auto var1 = std::make_shared<FactorVariable>(1, 3);
    auto var2 = std::make_shared<FactorVariable>(2, 3);
    BinaryFactor f2(var1, var2, A);

    EXPECT_EQ(1u, f2.variables[0]->id);
    EXPECT_EQ(2u, f2.variables[1]->id);
    EXPECT_TRUE(f2.measurement.isApprox(expected));
    std::cout << f2 << std::endl;
}

}  // end of wave namespace
