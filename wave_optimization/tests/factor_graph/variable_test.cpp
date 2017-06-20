#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"


namespace wave {

TEST(ValueTest, constructFromRawData) {
    // Construct a variable from raw buffer, as ceres would give
    double buf[3] = {7.7, 8.8, 9.9};
    auto expected = Vec3{7.7, 8.8, 9.9};

    auto val = ValueView<3>{buf};
    static_assert(3u == ValueView<3>::Size, "");
    EXPECT_EQ(3, val.size());

    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec3>{val.data()});
}

TEST(ValueTest, print) {
    double buf[2] = {7.7, 8.8};
    auto val = ValueView<2>{buf};
    std::stringstream ss, ss2;
    ss << val;
    EXPECT_STREQ("ValueView<2>", ss.str().c_str());

    val.print(ss2);
    EXPECT_EQ(ss.str(), ss2.str());
}

TEST(VariableTest, constructDefault) {
    // Construct an uninitialized variable
    auto var = FactorVariable<ValueView<2>>{};
    EXPECT_EQ(2, var.size());
}

TEST(VariableTest, constructInitialValue) {
    // Construct and initialize a variable
    const auto expected = Vec2{1.1, 2.2};
    auto var = FactorVariable<ValueView<2>>{expected};
    EXPECT_EQ(2, var.size());
    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec2>{var.value.data()});
}

TEST(VariableTest, constructInitialRvalue) {
    // Construct and initialize a variable
    auto var = FactorVariable<ValueView<2>>{Vec2{1.1, 2.2}};
    const auto expected = Vec2{1.1, 2.2};
    EXPECT_EQ(2, var.size());
    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec2>{var.value.data()});
}

TEST(VariableTest, copyConstruct) {
    // Because FactorVariable works with pointers to its storage, it must have
    // a custom copy constructor
    FactorVariable<ValueView<2>> a{Vec2{1.1, 2.2}};
    FactorVariable<ValueView<2>> b{a};

    ASSERT_NE(&a, &b);
    EXPECT_NE(a.data(), b.data());
    EXPECT_DOUBLE_EQ(1.1, *a.data());
    EXPECT_DOUBLE_EQ(1.1, *b.data());
}

TEST(VariableTest, moveConstruct) {
    FactorVariable<ValueView<2>> a;
    auto a_ptr = a.data();
    FactorVariable<ValueView<2>> b{std::move(a)};

    ASSERT_NE(&a, &b);
    EXPECT_NE(a_ptr, b.data());
}

TEST(VariableTest, copyAssign) {
    FactorVariable<ValueView<2>> a, b;
    b = a;

    ASSERT_NE(&a, &b);
    EXPECT_NE(a.data(), b.data());
}


TEST(VariableTest, moveAssign) {
    FactorVariable<ValueView<2>> a, b;
    b = std::move(a);

    ASSERT_NE(&a, &b);
    EXPECT_NE(a.data(), b.data());
}

}  // namespace wave
