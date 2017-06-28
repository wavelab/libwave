#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorMeasurement.hpp"

namespace wave {

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

TEST(MeasurementTest, constructFromRvalue) {
    auto meas = FactorMeasurement<FactorValue2>{Vec2{1.1, 2.2}, Vec2{0.1, 0.2}};
    const auto expected_val = Vec2{1.1, 2.2};
    //    EXPECT_EQ(2, meas.size());
    EXPECT_PRED2(
      VectorsNear, expected_val, Eigen::Map<Vec2>{meas.value.data()});
}

}  // namespace wave
