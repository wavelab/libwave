#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorValue.hpp"

namespace wave {

TEST(FactorValue, construct) {
    // Construct a FactorValue from an Eigen Vector, copying it
    auto vec = Vec3{7.7, 8.8, 9.9};
    auto val = FactorValue<double, void, 3>{vec};
    EXPECT_NE(vec.data(), val.data());
    EXPECT_EQ(vec, val);
    EXPECT_PRED2(VectorsNear, vec, val);
}

TEST(FactorValue, constructDouble) {
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

TEST(FactorValue, constructMap) {
    // Construct a mapped value mapping a raw buffer
    double buf[3] = {7.7, 8.8, 9.9};
    auto expected = Vec3{7.7, 8.8, 9.9};

    auto val = FactorValue<double, FactorValueOptions::Map, 3>{buf};
    static_assert(3u == FactorValue3<double>::SizeAtCompileTime, "");
    EXPECT_EQ(3, val.size());
    EXPECT_EQ(buf, val.data());
    EXPECT_PRED2(VectorsNear, expected, val);
}

TEST(FactorValue, print) {
    double buf[2] = {7.7, 8.8};
    auto val = FactorValue2<double>{buf};
    std::stringstream ss;
    ss << val;

    // For now, the result matches printing an Eigen vector
    EXPECT_STREQ("7.7\n8.8", ss.str().c_str());
}

}  // namespace wave
