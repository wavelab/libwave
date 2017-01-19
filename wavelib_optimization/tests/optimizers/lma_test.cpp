#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/benchmark.hpp"
#include "slam/optimization/optimizers/lma.hpp"


std::vector<slam::MatX> generate_data(void)
{
    slam::VecX input;
    slam::VecX beta;

    std::vector<double> inputs;
    std::vector<double> outputs;

    std::vector<slam::MatX> data;

    // setup
    input.resize(2);
    beta.resize(2);

    beta(0) = 1.0;
    beta(1) = 100.0;

    // generate data
    for (double x = -3.5; x < 3.5; x += 0.1) {
        for (double y = -3.5; y < 3.5; y += 0.1) {
            input << x, y;
            inputs.push_back(input(0));
            inputs.push_back(input(1));
            outputs.push_back(slam::rosenbrock(input, beta));
        }
    }

    // convert data to matrices and vectors
    slam::MatX x;
    slam::VecX y;

    y.resize(outputs.size());
    for (int i = 0; i < outputs.size(); i++) {
        y(i) = outputs[i];
    }

    x.resize(inputs.size() / 2.0, 2);
    for (int i = 0; i < (inputs.size() / 2.0); i++) {
        x(i, 0) = inputs[2 * i];
        x(i, 1) = inputs[2 * i + 1];
    }

    data.push_back(x);
    data.push_back(y);
    data.push_back(beta);

    return data;
}

void test_settings(slam::LMASettings &settings)
{
    std::vector<slam::MatX> data;
    slam::MatX x;

    settings.max_iter = 100;
    settings.lambda = 0.01;
    settings.function = LMA_BIND(slam::rosenbrock);
    settings.jacobian = LMA_BIND(slam::rosenbrock_jacobian);
    settings.nb_inputs = 2;
    settings.nb_params = 2;

    data = generate_data();
    settings.x = data[0];
    settings.y = data[1];
    settings.beta = data[2];
}

TEST(LMAOpt, constructor)
{
    slam::LMAOpt opt;

    ASSERT_EQ(false, opt.configured);
    ASSERT_EQ(100, opt.max_iter);
    ASSERT_FLOAT_EQ(0.01, opt.lambda);
    ASSERT_EQ(nullptr, opt.function);
    ASSERT_EQ(nullptr, opt.jacobian);
    ASSERT_EQ(0, opt.nb_inputs);
    ASSERT_EQ(0, opt.nb_params);

    ASSERT_FLOAT_EQ(0.0, opt.x(0));
    ASSERT_FLOAT_EQ(0.0, opt.y(0));
    ASSERT_FLOAT_EQ(0.0, opt.beta(0));

    ASSERT_FLOAT_EQ(0.0, opt.y_est(0));
    ASSERT_FLOAT_EQ(0.0, opt.diff(0));

    ASSERT_FLOAT_EQ(0.0, opt.J(0));
    ASSERT_FLOAT_EQ(0.0, opt.H(0));

    ASSERT_EQ(FLT_MAX, opt.error);
}

TEST(LMAOpt, configure)
{
    slam::LMAOpt opt;
    slam::LMASettings settings;

    test_settings(settings);
    opt.configure(settings);

    ASSERT_EQ(true, opt.configured);
    ASSERT_EQ(settings.max_iter, opt.max_iter);
    ASSERT_FLOAT_EQ(settings.lambda, opt.lambda);
    ASSERT_NE(nullptr, opt.function);
    ASSERT_NE(nullptr, opt.jacobian);
    ASSERT_EQ(settings.nb_inputs, opt.nb_inputs);
    ASSERT_EQ(settings.nb_params, opt.nb_params);

    ASSERT_FLOAT_EQ(settings.x(0), opt.x(0));
    ASSERT_FLOAT_EQ(settings.y(0), opt.y(0));
    ASSERT_FLOAT_EQ(settings.beta(0), opt.beta(0));

    ASSERT_FLOAT_EQ(0.0, opt.y_est(0));
    ASSERT_FLOAT_EQ(0.0, opt.diff(0));

    ASSERT_FLOAT_EQ(0.0, opt.J(0, 0));
    ASSERT_FLOAT_EQ(0.0, opt.H(0, 0));

    ASSERT_EQ(FLT_MAX, opt.error);
}

TEST(LMAOpt, evalFunction)
{
    slam::LMAOpt opt;
    slam::LMASettings settings;
    double error;

    // configure
    test_settings(settings);
    opt.configure(settings);

    // test and assert
    opt.evalFunction(opt.beta, error);
    ASSERT_FLOAT_EQ(0.0, error);
}

TEST(LMAOpt, calcGradients)
{
    slam::LMAOpt opt;
    slam::LMASettings settings;
    slam::MatX J_before, H_before;

    // configure
    test_settings(settings);
    opt.configure(settings);
    J_before = opt.J;
    H_before = opt.H;

    // test and assert
    opt.calcGradients(opt.beta);

    ASSERT_FALSE(J_before.isApprox(opt.J));
    ASSERT_FALSE(H_before.isApprox(opt.H));
}

TEST(LMAOpt, iterate)
{
    slam::LMAOpt opt;
    slam::LMASettings settings;
    slam::VecX beta_before;
    double error;
    std::vector<slam::VecX> data;

    // configure
    test_settings(settings);
    opt.configure(settings);

    opt.beta << 1.0, 90.0;
    beta_before = opt.beta;

    // test and assert
    opt.evalFunction(opt.beta, opt.error);
    opt.calcGradients(opt.beta);

    std::cout << "beta: " << opt.beta.transpose() << std::endl;
    opt.iterate();
    std::cout << "beta: " << opt.beta.transpose() << std::endl;

    ASSERT_FALSE(beta_before.isApprox(opt.beta));
}

TEST(LMAOpt, optimize)
{
    slam::VecX beta;
    slam::LMAOpt opt;
    slam::LMASettings settings;
    std::vector <slam::VecX> data;

    // configure
    test_settings(settings);
    settings.max_iter = 100;
    settings.beta << 1.01, 99.99;
    opt.configure(settings);

    opt.optimize();
    std::cout << opt.beta.transpose() << std::endl;
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
