#ifndef __SLAM_OPTIMIZATION_BENCHMARK_HPP__
#define __SLAM_OPTIMIZATION_BENCHMARK_HPP__

#include "slam/utils/utils.hpp"
#include "slam/optimization/benchmark.hpp"

namespace slam {

double ackley(VecX x);
double beale(VecX x);
double booth(VecX x);
double matyas(VecX x);
double sphere(VecX x);
double rosenbrock(VecX x, VecX beta);
VecX rosenbrock_jacobian(VecX x, VecX beta);

}  // end of slam namespace
#endif
