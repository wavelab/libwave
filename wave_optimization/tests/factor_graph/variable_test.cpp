#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorMeasurement.hpp"

namespace wave {

TEST(ValueTest, constructFromRawData) {
    // Construct a variable from raw buffer, as ceres would give
    double buf[3] = {7.7, 8.8, 9.9};
    auto expected = Vec3{7.7, 8.8, 9.9};

    auto val = FactorValue3<double>{buf};
    static_assert(3u == FactorValue3<double>::SizeAtCompileTime, "");
    EXPECT_EQ(3, val.size());

    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec3>{val.data()});
}

TEST(ValueTest, print) {
    double buf[2] = {7.7, 8.8};
    auto val = FactorValue2<double>{buf};
    std::stringstream ss;
    ss << val;

    // For now, the result matches printing an Eigen vector
    EXPECT_STREQ("7.7\n8.8", ss.str().c_str());
}

TEST(VariableTest, constructDefault) {
    // Construct an uninitialized variable
    auto var = FactorVariable<FactorValue2>{};
}

TEST(VariableTest, constructInitialValue) {
    // Construct and initialize a variable
    const auto expected = Vec2{1.1, 2.2};
    auto var = FactorVariable<FactorValue2>{expected};
    //    EXPECT_EQ(2, var.size());
    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec2>{var.value.data()});
}

TEST(VariableTest, constructInitialRvalue) {
    // Construct and initialize a variable
    auto var = FactorVariable<FactorValue2>{Vec2{1.1, 2.2}};
    const auto expected = Vec2{1.1, 2.2};
    //    EXPECT_EQ(2, var.size());
    EXPECT_PRED2(VectorsNear, expected, Eigen::Map<Vec2>{var.value.data()});
}
//
// TEST(VariableTest, copyConstruct) {
//    // Because FactorVariable works with pointers to its storage, it must have
//    // a custom copy constructor
//    FactorVariable<FactorValue2> a{Vec2{1.1, 2.2}};
//    FactorVariable<FactorValue2> b{a};
//
//    ASSERT_NE(&a, &b);
//    EXPECT_NE(a.data(), b.data());
//    EXPECT_DOUBLE_EQ(1.1, *a.data());
//    EXPECT_DOUBLE_EQ(1.1, *b.data());
//}
//
// TEST(VariableTest, moveConstruct) {
//    FactorVariable<FactorValue2> a;
//    auto a_ptr = a.data();
//    FactorVariable<FactorValue2> b{std::move(a)};
//
//    ASSERT_NE(&a, &b);
//    EXPECT_NE(a_ptr, b.data());
//}
//
// TEST(VariableTest, copyAssign) {
//    FactorVariable<FactorValue2> a, b;
//    b = a;
//
//    ASSERT_NE(&a, &b);
//    EXPECT_NE(a.data(), b.data());
//}
//
//
// TEST(VariableTest, moveAssign) {
//    FactorVariable<FactorValue2> a, b;
//    b = std::move(a);
//
//    ASSERT_NE(&a, &b);
//    EXPECT_NE(a.data(), b.data());
//}

TEST(MeasurementTest, constructFromRvalue) {
    auto meas = FactorMeasurement<FactorValue2>{Vec2{1.1, 2.2}, Vec2{0.1, 0.2}};
    const auto expected_val = Vec2{1.1, 2.2};
    //    EXPECT_EQ(2, meas.size());
    EXPECT_PRED2(
      VectorsNear, expected_val, Eigen::Map<Vec2>{meas.value.data()});
}

}  // namespace wave
