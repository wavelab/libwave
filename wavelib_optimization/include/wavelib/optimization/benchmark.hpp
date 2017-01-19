#ifndef __wavelib_OPTIMIZATION_BENCHMARK_HPP__
#define __wavelib_OPTIMIZATION_BENCHMARK_HPP__

#include "wavelib/utils/utils.hpp"
#include "wavelib/optimization/benchmark.hpp"

namespace wavelib {

double ackley(VecX x);
double beale(VecX x);
double booth(VecX x);
double matyas(VecX x);
double sphere(VecX x);
double rosenbrock(VecX x, VecX beta);
VecX rosenbrock_jacobian(VecX x, VecX beta);

}  // end of wavelib namespace
#endif
