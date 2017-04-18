#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/variable.hpp"


namespace wave {

TEST(Variable, constructor) {
    Variable var1(VariableType::POSE, 1);
    EXPECT_EQ(VariableType::POSE, var1.type);
    EXPECT_EQ(1u, var1.id);

    Variable var2(VariableType::LANDMARK, 2);
    EXPECT_EQ(VariableType::LANDMARK, var2.type);
    EXPECT_EQ(2u, var2.id);
}

TEST(PoseVar, constructor) {
    PoseVar var(1);

    EXPECT_EQ(VariableType::POSE, var.type);
    EXPECT_EQ(1u, var.id);
}

TEST(LandmarkVar, constructor) {
    LandmarkVar var(2);

    EXPECT_EQ(VariableType::LANDMARK, var.type);
    EXPECT_EQ(2u, var.id);
}

}  // end of wave namespace
