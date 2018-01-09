#include <ceres/gradient_checker.h>
#include <Eigen/Eigenvalues>

#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/transform_prior.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"

namespace wave {

TEST(transform_prior, unit_weights) {
    const double **trans;
    trans = new const double *[1];
    trans[0] = new const double[12]{0.999613604886095,
                                    0.027796419313034,
                                    0,
                                    -0.027796419313034,
                                    0.999613604886095,
                                    0,
                                    0,
                                    0,
                                    1,
                                    3.599536313918120,
                                    0.050036777340220,
                                    0};

    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[72];

    double residual[6] = {0};
    Transformation prior_val;

    ceres::CostFunction* cost_function = new TransformPrior(Mat6::Identity(), prior_val);
    cost_function->Evaluate(trans, residual, jacobian);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1.1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

TEST(transform_prior, weighted_information) {
    const double **trans;
    trans = new const double *[1];
    trans[0] = new const double[12]{0.999613604886095,
                                    0.027796419313034,
                                    0,
                                    -0.027796419313034,
                                    0.999613604886095,
                                    0,
                                    0,
                                    0,
                                    1,
                                    3.599536313918120,
                                    0.050036777340220,
                                    0};

    double **jacobian;
    jacobian = new double *[1];
    jacobian[0] = new double[72];

    double residual[6] = {0};
    Transformation prior_val;

    Mat6 covar;
    covar << 0.049608482448035, 0.000039527111973, -0.000079054223946, -0.000002281574415, 0.014183675367405, -0.026870388557460,
            0.000039527111973, 0.049667773115995, -0.000158108447892, -0.013392676813061, 0.000590967341343, 0.015158324421416,
            -0.000079054223946, -0.000158108447892, 0.049904935787833, 0.026276569248098, -0.016345963040141, -0.000988748192931,
            -0.000002281574415, -0.013392676813061, 0.026276569248098, 0.067465448431922, -0.008962627787929, -0.004834749671968,
            0.014183675367405, 0.000590967341343, -0.016345963040141, -0.008962627787929, 0.059376860688103, -0.007535954876835,
            -0.026870388557460, 0.015158324421416, -0.000988748192931, -0.004834749671968, -0.007535954876835, 0.069674001585200;

    Eigen::SelfAdjointEigenSolver<Mat6> es(covar.inverse());
    Mat6 sqrtinfo = es.operatorSqrt();

    ceres::CostFunction* cost_function = new TransformPrior(sqrtinfo, prior_val);
    cost_function->Evaluate(trans, residual, jacobian);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 5e-5, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

}