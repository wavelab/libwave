#include <gtest/gtest.h>

#include "slam/symmath/models.hpp"
#include "slam/symmath/symmath.hpp"


TEST(Symmath, output_jacobian)
{
    std::vector<GiNaC::ex> model;
    std::vector<GiNaC::symbol> states;

    slam::quadrotor_jacobian(model, states);
    slam::output_jacobian("/tmp/model.dat", model, states);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
