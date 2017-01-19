#include <gtest/gtest.h>

#include "slam/symmath/models.hpp"


TEST(Models, bundle_adjustment_jacobian)
{
    slam::bundle_adjustment_jacobian("/tmp/model.dat");
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
