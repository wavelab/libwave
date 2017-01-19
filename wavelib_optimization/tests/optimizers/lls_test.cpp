#include <gtest/gtest.h>

#include "slam/utils/math.hpp"
#include "slam/optimization/optimizers/lls.hpp"




TEST(LLSSolver, constructor)
{
    slam::LLSSolver solver;

    ASSERT_EQ(solver.configured, false);
}

TEST(LLSSolver, configure)
{
    slam::LLSSolver solver;

    solver.configure();
    ASSERT_EQ(solver.configured, true);
}

TEST(LLSSolver, solve)
{
    slam::LLSSolver solver;
    slam::MatX A(4, 2);
    slam::VecX b(4);
    slam::VecX x;

    A << 1, 1,
         1, 2,
         1, 3,
         1, 4;

    b << 6,
         5,
         7,
         10;

    solver.configure();
    solver.solve(A, b, x);

    ASSERT_FLOAT_EQ(x(0), 3.5);
    ASSERT_FLOAT_EQ(x(1), 1.4);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
