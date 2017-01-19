#ifndef __wave_OPTIMIZATION_BENCHMARK_HPP__
#define __wave_OPTIMIZATION_BENCHMARK_HPP__

#include "wave/utils/utils.hpp"
#include "wave/optimization/benchmark.hpp"

namespace wave {

double ackley(VecX x);
double beale(VecX x);
double booth(VecX x);
double matyas(VecX x);
double sphere(VecX x);
double rosenbrock(VecX x, VecX beta);
VecX rosenbrock_jacobian(VecX x, VecX beta);

}  // end of wave namespace
#endif
