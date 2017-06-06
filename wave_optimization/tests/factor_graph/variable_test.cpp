#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"


namespace wave {

TEST(VariableTest, constructFromData) {
    double in[] = {1.1, 2.2};
    auto var = FactorVariable<2>{in};
    static_assert(2 == FactorVariable<2>::SizeAtCompileTime, "");

    EXPECT_EQ(in, var.data());
}

TEST(VariableTest, constructDefault) {
    auto var = FactorVariable<2>{};
    EXPECT_EQ(nullptr, var.data());
}

TEST(VariableTest, print) {
    auto var = FactorVariable<2>{};
    std::stringstream ss, ss2;
    ss << var;
    EXPECT_STREQ("FactorVariable<2>", ss.str().c_str());

    var.print(ss2);
    EXPECT_EQ(ss.str(), ss2.str());
}

}  // namespace wave
