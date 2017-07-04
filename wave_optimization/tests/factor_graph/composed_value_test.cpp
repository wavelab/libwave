#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/ComposedValue.hpp"

namespace wave {
struct ComposedValueTest : public ::testing::Test {
    // Define a sample composed value (use aliases for sized FactorValue)
    template <typename T, typename O = void>
    struct Composed : ComposedValue<Composed<T, O>,
                                    FactorValue1,
                                    FactorValue2,
                                    FactorValue3> {
        using Base = ComposedValue<Composed<T, O>,
                                   FactorValue1,
                                   FactorValue2,
                                   FactorValue3>;
        using Base::Base;
        Composed() = default;
        // The copy constructor must not copy the reference members
        Composed(const Composed &rhs) : Base{rhs} {}
        Ref<FactorValue<T, O, 1>> b0 = this->template block<0>();
        Ref<FactorValue<T, O, 2>> b1 = this->template block<1>();
        Ref<FactorValue<T, O, 3>> b2 = this->template block<2>();
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
    auto res = c.data();
    auto expected = c.b0.data();
    EXPECT_EQ(expected, res);
}

TEST_F(ComposedValueTest, constructMapped) {
    auto a0 = 1.1;
    auto a1 = Vec2{2.2, 3.3};
    auto a2 = Vec3{4.4, 5.5, 6.6};

    Vec6 buf;
    buf << a0, a1, a2;

    auto c = Composed<double, FactorValueOptions::Map>{buf.data()};
    EXPECT_EQ(a0, c.b0.value());
    EXPECT_EQ(a1, c.b1);
    EXPECT_EQ(a2, c.b2);
    EXPECT_EQ(buf.data(), c.b0.data());
    EXPECT_EQ(buf.data() + 1, c.b1.data());
    EXPECT_EQ(buf.data() + 3, c.b2.data());

    static_assert(
      std::is_same<Eigen::Ref<FactorValue<double, FactorValueOptions::Map, 1>>,
                   decltype(c.b0)>::value,
      "");
}

TEST_F(ComposedValueTest, subtractCompound) {
    auto a0 = 1.1;
    auto a1 = Vec2{2.2, 3.3};
    auto a2 = Vec3{4.4, 5.5, 6.6};
    auto b0 = 0.9;
    auto b1 = Vec2{0.8, 0.7};
    auto b2 = Vec3{0.6, 0.5, 0.4};

    auto a = Composed<double>{a0, a1, a2};
    const auto b = Composed<double>{b0, b1, b2};
    a -= b;

    EXPECT_DOUBLE_EQ(a0 - b0, a.b0[0]);
    EXPECT_EQ(a1 - b1, a.b1);
    EXPECT_EQ(a2 - b2, a.b2);
}

TEST_F(ComposedValueTest, subtract) {
    auto a0 = 1.1;
    auto a1 = Vec2{2.2, 3.3};
    auto a2 = Vec3{4.4, 5.5, 6.6};
    auto b0 = 0.9;
    auto b1 = Vec2{0.8, 0.7};
    auto b2 = Vec3{0.6, 0.5, 0.4};

    const auto a = Composed<double>{a0, a1, a2};
    const auto b = Composed<double>{b0, b1, b2};
    auto c = a - b;

    EXPECT_DOUBLE_EQ(a0 - b0, c.b0[0]);
    EXPECT_EQ(a1 - b1, c.b1);
    EXPECT_EQ(a2 - b2, c.b2);
}

// Test the Square option
// Produces a ComposedValue with a square block for each nested value
TEST_F(ComposedValueTest, constructSquare) {
    Composed<double, FactorValueOptions::Square> c{};
    EXPECT_EQ(1, c.b0.size());
    EXPECT_EQ(4, c.b1.size());
    EXPECT_EQ(9, c.b2.size());
}

TEST_F(ComposedValueTest, indexFromRef) {
    auto other = Composed<double, FactorValueOptions::Square>{};
    auto c = Composed<double, FactorValueOptions::Square>{};
    EXPECT_EQ(0, c.indexFromRef(c.b0));
    EXPECT_EQ(1, c.indexFromRef(c.b1));
    EXPECT_EQ(3, c.indexFromRef(c.b2));

    // Throw on out-of-bounds input
    EXPECT_THROW(c.indexFromRef(other.b0), std::logic_error);
    EXPECT_THROW(c.indexFromRef(other.b2), std::logic_error);
    EXPECT_THROW(other.indexFromRef(c.b0), std::logic_error);
    EXPECT_THROW(other.indexFromRef(c.b2), std::logic_error);
}

}  // namespace wave
