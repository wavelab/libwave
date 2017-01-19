#include <gtest/gtest.h>

#include "wave/symmath/models.hpp"


TEST(Models, bundle_adjustment_jacobian)
{
    wave::bundle_adjustment_jacobian("/tmp/model.dat");
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
