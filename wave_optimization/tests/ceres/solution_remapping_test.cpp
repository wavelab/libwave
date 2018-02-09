#include <ceres/gradient_checker.h>
#include <ceres/normal_prior.h>

#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/solution_remapping_parameterization.hpp"

namespace wave {

TEST(subset, sixdof) {
    Mat6 eye = Mat6::Identity();
    Vec6 zero = Vec6::Zero();
    Vec6 prob_vec;
    prob_vec.setZero();
    ceres::CostFunction *cost_function = new ceres::NormalPrior(eye, zero);

    Mat6 projection_mat;
    // I wonder if a strange matrix will break something?
    projection_mat.setRandom();

    ceres::LocalParameterization *remap_param = new RemapParameterization<6>(projection_mat);
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(remap_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    const double **params;
    params = new const double *;
    params[0] = prob_vec.data();
    EXPECT_TRUE(g_check.Probe(params, 1e-8, &g_results));
}

}

