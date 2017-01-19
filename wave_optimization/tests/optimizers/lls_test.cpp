#include <gtest/gtest.h>

#include "wave/utils/math.hpp"
#include "wave/optimization/optimizers/lls.hpp"




TEST(LLSSolver, constructor)
{
    wave::LLSSolver solver;

    ASSERT_EQ(solver.configured, false);
}

TEST(LLSSolver, configure)
{
    wave::LLSSolver solver;

    solver.configure();
    ASSERT_EQ(solver.configured, true);
}

TEST(LLSSolver, solve)
{
    wave::LLSSolver solver;
    wave::MatX A(4, 2);
    wave::VecX b(4);
    wave::VecX x;

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
