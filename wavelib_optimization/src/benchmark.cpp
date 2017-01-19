#include "slam/optimization/benchmark.hpp"


namespace slam {

double ackley(VecX x)
{
    // range: -5.0 <= x, y <= 5.0
    // minimum: x = 0.0, y = 0.0
    return -20 * exp(-0.2 * sqrt(0.5 * ((x(0) * x(0)) + (x(1) * x(1)))))
           - exp(0.5 * (cos(2 * M_PI * x(0)) + cos(2 * M_PI * x(1))))
           + exp(1.0) + 20;
}

double beale(VecX x)
{
    // range: -4.5 <= x, y <= 4.5
    // minimum: x = 3.0, y = 0.5
    return pow((1.5 - x(0) + x(0) * x(1)), 2)
           + pow((2.25 - x(0) + x(0) * pow(x(1), 2)), 2)
           + pow((2.625 - x(0) + x(0) * pow(x(1), 3)), 2);
}

double booth(VecX x)
{
    // range: -10.0 <= x, y <= 10.0
    // minimum: x = 1.0, y = 3.0
    return pow(x(0) + 2 * x(1) - 7, 2) + pow(2 * x(0) + x(1) - 5, 2);
}

double matyas(VecX x)
{
    // range: -10.0 <= x, y <= 10.0
    // minimum: x = 0.0, y = 0.0
    return 0.26 * (pow(x(0), 2) + pow(x(1), 2)) - 0.48 * x(0) * x(1);
}

double sphere(VecX x)
{
    double sum;

    // range: -infinity <= x(n) <= infinity
    // minimum: x(n) = 0.0
    sum = 0.0;
    for (int i = 0; i < x.rows(); i++) {
        sum += pow(x(i), 2);
    }

    return sum;
}

double rosenbrock(VecX x, VecX beta)
{
    double a, b;

    a = beta(0);
    b = beta(1);

    // minimum at (x, y) = (a, a^{2}), where f(x, y) = 0
    return pow(a - x(0), 2) + b * pow(x(1) - pow(x(0), 2), 2);
}

VecX rosenbrock_jacobian(VecX x, VecX beta)
{
    double a;
    double b;
    slam::VecX J;

    // setup
    a = beta(0);
    b = beta(1);
    J.resize(2);

    // evaluate jacobian
    J(0) = 2.0 * x(0) * (2.0 * b * (pow(x(0), 2) - x(1)) - 1.0);
    J(1) = 2.0 * b * (x(1) - pow(x(0), 2));

    return J;
}

}  // end of slam namespace
