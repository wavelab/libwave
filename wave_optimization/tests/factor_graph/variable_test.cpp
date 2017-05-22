#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/variable.hpp"


namespace wave {

TEST(FactorVariable, constructor) {
    FactorVariable var(1, 2);
    EXPECT_EQ(1u, var.id);
    EXPECT_EQ(2u, var.data.size());

    std::cout << var << std::endl;
}

TEST(PoseVar, constructor) {
    PoseVar var(1);

    EXPECT_EQ(1u, var.id);
    EXPECT_EQ(6u, var.data.size());

    std::cout << var << std::endl;
}

TEST(LandmarkVar, constructor) {
    LandmarkVar var(2);

    EXPECT_EQ(2u, var.id);
    EXPECT_EQ(3u, var.data.size());

    std::cout << var << std::endl;
}

}  // end of wave namespace
