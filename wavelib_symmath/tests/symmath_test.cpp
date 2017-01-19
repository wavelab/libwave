#include <gtest/gtest.h>

#include "wavelib/symmath/models.hpp"
#include "wavelib/symmath/symmath.hpp"


TEST(Symmath, output_jacobian)
{
    std::vector<GiNaC::ex> model;
    std::vector<GiNaC::symbol> states;

    wavelib::quadrotor_jacobian(model, states);
    wavelib::output_jacobian("/tmp/model.dat", model, states);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
