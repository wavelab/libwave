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

TEST(FactorValue, constructSquare) {
    Mat2 mat;
    mat << 1.1, 2.2, 3.3, 4.4;
    auto val = FactorValue<double, FactorValueOptions::Square, 2>{mat};

    EXPECT_EQ(mat, val);
    EXPECT_NE(mat.data(), val.data());
    EXPECT_EQ(4u, val.size());
    EXPECT_EQ(2u, val.rows());
    EXPECT_EQ(2u, val.cols());
}

TEST(FactorValue, constructSquareSize1) {
    FactorValue<double, FactorValueOptions::Square, 1> val{1.1};

    EXPECT_EQ(1.1, val[0]);
}

TEST(FactorValue, assignSquareSize1) {
    FactorValue<double, FactorValueOptions::Square, 1> val;
    val = 2.3;

    EXPECT_EQ(2.3, val[0]);
}

TEST(FactorValue, initializeSquare) {
    Mat2 mat;
    mat << 1.1, 2.2, 3.3, 4.4;
    FactorValue<double, FactorValueOptions::Square, 2> val;
    val << 1.1, 2.2, 3.3, 4.4;

    EXPECT_EQ(mat, val);
    EXPECT_NE(mat.data(), val.data());
}

}  // namespace wave
