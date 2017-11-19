#ifndef WAVE_KERNELS_HPP
#define WAVE_KERNELS_HPP

// This file defines scoring kernels for use with odometry

namespace wave {

static double loam_kernel[11] = {1, 1, 1, 1, 1, -10, 1, 1, 1, 1, 1};
static double LoG_kernel[11] = {0.000232391821040,
                         0.001842097682135,
                         0.034270489647107,
                         0.166944943945706,
                         -0.009954755288609,
                         -0.386583206270711,
                         -0.009954755288609,
                         0.166944943945706,
                         0.034270489647107,
                         0.001842097682135,
                         0.000232391821040};

static double FoG_kernel[9] = {0.003571428, -0.0380952, 0.2, -0.8, 0, 0.8, -0.2, 0.0380952, -0.003571428};

}  // namespace wave

#endif //WAVE_KERNELS_HPP
