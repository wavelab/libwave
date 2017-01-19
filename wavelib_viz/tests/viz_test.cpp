#include <gtest/gtest.h>

#include "wavelib/viz/viz.hpp"




TEST(Viz, constructor)
{
    wavelib::Viz viz;

    ASSERT_EQ(false, viz.configured);
}

TEST(Viz, configure)
{
    wavelib::Viz viz;

    viz.configure();
    ASSERT_EQ(true, viz.configured);
}

TEST(Viz, run)
{
    wavelib::Viz viz;

    viz.configure();
    viz.run();
}


int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
