#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorValue.hpp"

namespace wave {

TEST(FactorValue, construct) {
    // Default construct an uninitialized FactorValue
    auto val = FactorValue<double, void, 3>{};
    EXPECT_EQ(3u, val.size());
}

TEST(FactorValue, constructFromVector) {
    // Construct a FactorValue from an Eigen Vector, copying it
    auto vec = Vec3{7.7, 8.8, 9.9};
    auto val = FactorValue<double, void, 3>{vec};
    EXPECT_NE(vec.data(), val.data());
    EXPECT_EQ(vec, val);
    EXPECT_PRED2(VectorsNear, vec, val);
}

TEST(FactorValue, constructFromList) {
    // Construct a size-1 FactorValue from a double
    auto val = FactorValue<double, void, 3>{1.1, 2.2, 3.3};
    EXPECT_EQ(Vec3(1.1, 2.2, 3.3), val);
}

TEST(FactorValue, constructFromScalar) {
    // Construct a size-1 FactorValue from a double
    auto val = FactorValue<double, void, 1>{1.1};
    EXPECT_EQ(1.1, val[0]);
}

TEST(FactorValue, constructFromRaw) {
    // Construct a FactorValue from a raw buffer, copying it
    double buf[3] = {7.7, 8.8, 9.9};
    auto expected = Vec3{7.7, 8.8, 9.9};
    auto val = FactorValue<double, void, 3>{buf};
    static_assert(3u == FactorValue3<double>::SizeAtCompileTime, "");
    EXPECT_EQ(3, val.size());
    EXPECT_NE(buf, val.data());
    EXPECT_EQ(expected, val);
    EXPECT_PRED2(VectorsNear, expected, val);
}

TEST(FactorValue, assign) {
    auto a = FactorValue<double, void, 2>{};
    auto b = FactorValue<double, void, 2>{1.2, 3.4};

    a = b;

    EXPECT_EQ(a, b);
    EXPECT_NE(a.data(), b.data());
}

TEST(FactorValue, assignFromScalar) {
    // Assign to a size-1 FactorValue from a double
    auto val = FactorValue<double, void, 1>{1.1};
    val = 2.2;
    EXPECT_EQ(2.2, val[0]);
}

TEST(FactorValue, print) {
    double buf[2] = {7.7, 8.8};
    auto val = FactorValue2<double>{buf};
    std::stringstream ss;
    ss << val;

    // For now, the result matches printing an Eigen vector
    EXPECT_STREQ("7.7\n8.8", ss.str().c_str());
}

TEST(FactorValue, constructMappedFromRaw) {
    // Construct a mapped value mapping a raw buffer
    double buf[3] = {7.7, 8.8, 9.9};
    auto expected = Vec3{7.7, 8.8, 9.9};

    auto val = FactorValue<double, FactorValueOptions::Map, 3>{buf};
    static_assert(3u == FactorValue3<double>::SizeAtCompileTime, "");
    EXPECT_EQ(3, val.size());
    EXPECT_EQ(buf, val.data());
    EXPECT_PRED2(VectorsNear, expected, val);
}


TEST(FactorValue, assignMapped) {
    auto a = FactorValue<double, void, 2>{};
    auto map = FactorValue<double, FactorValueOptions::Map, 2>{a.data()};
    auto b = FactorValue<double, void, 2>{1.2, 3.4};

    map = b;

    EXPECT_EQ(a, b);
    EXPECT_NE(a.data(), b.data());
}

TEST(FactorValue, assignMappedFromScalar) {
    // Assign to a size-1 FactorValue from a double
    double x = 0;
    auto val = FactorValue<double, FactorValueOptions::Map, 1>{&x};
    val = 2.2;
    EXPECT_EQ(2.2, val[0]);
}

struct ComposedValueTest : public ::testing::Test {
    // Define a sample composed value (use aliases for sized FactorValue)
    template <typename T, typename O = void>
    struct Composed
      : ComposedValue<T, O, FactorValue1, FactorValue2, FactorValue3> {
        using ComposedValue<T, O, FactorValue1, FactorValue2, FactorValue3>::
          ComposedValue;
        FactorValue<T, O, 1> &b0 = this->template block<0>();
        FactorValue<T, O, 2> &b1 = this->template block<1>();
        FactorValue<T, O, 3> &b2 = this->template block<2>();
    };
};


TEST_F(ComposedValueTest, construct) {
    Composed<double>{};
}

TEST_F(ComposedValueTest, constructFromLists) {
    auto c = Composed<double>{1.1, {2.2, 3.3}, {4.4, 5.5, 6.6}};
    EXPECT_EQ(1.1, c.b0.value());
    EXPECT_EQ(Vec2(2.2, 3.3), c.b1);
    EXPECT_EQ(Vec3(4.4, 5.5, 6.6), c.b2);
}

TEST_F(ComposedValueTest, getData) {
    auto c = Composed<double>{1.1, {2.2, 3.3}, {4.4, 5.5, 6.6}};
    auto res = c.blockData();
    auto expected =
      std::vector<double *>{c.b0.data(), c.b1.data(), c.b2.data()};
    EXPECT_EQ(expected, res);
}

TEST_F(ComposedValueTest, constructMapped) {
    auto a0 = 1.1;
    auto a1 = Vec2{2.2, 3.3};
    auto a2 = Vec3{4.4, 5.5, 6.6};

    auto c =
      Composed<double, FactorValueOptions::Map>{&a0, a1.data(), a2.data()};
    EXPECT_EQ(a0, c.b0.value());
    EXPECT_EQ(a1, c.b1);
    EXPECT_EQ(a2, c.b2);
    EXPECT_EQ(&a0, c.b0.data());
    EXPECT_EQ(a1.data(), c.b1.data());
    EXPECT_EQ(a2.data(), c.b2.data());

    static_assert(
      std::is_same<FactorValue<double, FactorValueOptions::Map, 1> &,
                   decltype(c.b0)>::value,
      "");
}

}  // namespace wave
