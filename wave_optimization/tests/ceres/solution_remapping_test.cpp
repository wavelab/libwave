#include <ceres/gradient_checker.h>
#include <ceres/normal_prior.h>

#include "wave/wave_test.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/optimization/ceres/local_params/solution_remapping_parameterization.hpp"
#include "wave/optimization/ceres/local_params/trajectory_remap.hpp"

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

/**
 * Test is done around identity so that SE3 jacobian is identity
 */
TEST(trajectory, thirtydof) {
    double memblock[30];
    const double **params;
    params = new const double *;
    params[0] = memblock;

    Eigen::Map<Vec6> vel(memblock);
    Eigen::Map<Mat34> tk_map(memblock + 6, 3, 4);
    Eigen::Map<Mat34> tkp1_map(memblock + 18, 3, 4);
    Transformation<Eigen::Map<Mat34>, false> Tk(tk_map);
    Transformation<Eigen::Map<Mat34>, false> Tkp1(tkp1_map);

    vel.setZero();
    Tk.setIdentity();
    Tkp1.setIdentity();

    Eigen::Matrix<double, 18, 18> projection_mat;
    projection_mat.setRandom();

    ceres::LocalParameterization *remap_param = new TrajectoryParamRemap<30>(projection_mat);
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(remap_param);

    Eigen::Matrix<double, 30, 30> eye;
    eye.setIdentity();
    Eigen::Matrix<double, 30, 1> zero;
    zero.setZero();

    ceres::CostFunction *cost_function = new ceres::NormalPrior(eye, zero);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;

    EXPECT_TRUE(g_check.Probe(params, 1e-8, &g_results));
}

}

