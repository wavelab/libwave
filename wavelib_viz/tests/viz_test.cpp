#include <gtest/gtest.h>

#include "slam/viz/viz.hpp"




TEST(Viz, constructor)
{
    slam::Viz viz;

    ASSERT_EQ(false, viz.configured);
}

TEST(Viz, configure)
{
    slam::Viz viz;

    viz.configure();
    ASSERT_EQ(true, viz.configured);
}

TEST(Viz, run)
{
    slam::Viz viz;

    viz.configure();
    viz.run();
}


int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
