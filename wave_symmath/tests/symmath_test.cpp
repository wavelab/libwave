#include <gtest/gtest.h>

#include "wave/symmath/models.hpp"
#include "wave/symmath/symmath.hpp"


TEST(Symmath, output_jacobian)
{
    std::vector<GiNaC::ex> model;
    std::vector<GiNaC::symbol> states;

    wave::quadrotor_jacobian(model, states);
    wave::output_jacobian("/tmp/model.dat", model, states);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
