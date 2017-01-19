#include <gtest/gtest.h>

#include "wavelib/utils/math.hpp"
#include "wavelib/optimization/optimizers/lls.hpp"




TEST(LLSSolver, constructor)
{
    wavelib::LLSSolver solver;

    ASSERT_EQ(solver.configured, false);
}

TEST(LLSSolver, configure)
{
    wavelib::LLSSolver solver;

    solver.configure();
    ASSERT_EQ(solver.configured, true);
}

TEST(LLSSolver, solve)
{
    wavelib::LLSSolver solver;
    wavelib::MatX A(4, 2);
    wavelib::VecX b(4);
    wavelib::VecX x;

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
