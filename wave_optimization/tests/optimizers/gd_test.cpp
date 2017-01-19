#include <functional>

#include <gtest/gtest.h>

#include "wave/optimization/benchmark.hpp"
#include "wave/optimization/optimizers/gd.hpp"


TEST(GDOpt, constructor)
{
    wave::GDOpt opt;
    ASSERT_EQ(opt.configured, false);
}

TEST(GDOpt, configure)
{
    int max_iter;
    wave::VecX eta(2);
    wave::VecX x(2);
    wave::GDOpt opt;

    max_iter = 1000;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(wave::beale, std::placeholders::_1)
    );

    ASSERT_EQ(true, opt.configured);
    ASSERT_EQ(max_iter, opt.max_iter);
    ASSERT_FLOAT_EQ(eta(0), opt.eta(0));
    ASSERT_FLOAT_EQ(eta(1), opt.eta(1));
    ASSERT_FLOAT_EQ(x(0), opt.x(0));
    ASSERT_FLOAT_EQ(x(1), opt.x(1));
}

TEST(GDOpt, calcGradient)
{
    int max_iter;
    wave::VecX eta(2);
    wave::VecX x(2);
    wave::VecX df(2, 1);
    wave::GDOpt opt;

    max_iter = 1000;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(wave::beale, std::placeholders::_1)
    );
    opt.calcGradient(df);
    // std::cout << df << std::endl;

    ASSERT_FLOAT_EQ(-12.75, df(0));
    ASSERT_FLOAT_EQ(0.0, df(1));
}

TEST(GDOpt, optimize)
{
    int max_iter;
    wave::VecX eta(2);
    wave::VecX x(2);
    wave::GDOpt opt;

    max_iter = 10000;
    eta << 0.006, 0.006;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(wave::beale, std::placeholders::_1)
    );
    opt.optimize();
    // std::cout << opt.x << std::endl;

    ASSERT_TRUE(opt.x(0) > 2.7);
    ASSERT_TRUE(opt.x(0) <= 3.0);
    ASSERT_TRUE(opt.x(1) > 0.4);
    ASSERT_TRUE(opt.x(1) <= 0.5);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
