#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/variable.hpp"


namespace wave {

TEST(Variable, constructor) {
    Variable var1(1, 2);
    EXPECT_EQ(1u, var1.id);
    EXPECT_EQ(2u, var1.size);
}

TEST(PoseVar, constructor) {
    PoseVar var(1);

    EXPECT_EQ(1u, var.id);
    EXPECT_EQ(6u, var.size);
}

TEST(LandmarkVar, constructor) {
    LandmarkVar var(2);

    EXPECT_EQ(2u, var.id);
    EXPECT_EQ(3u, var.size);
}

}  // end of wave namespace
