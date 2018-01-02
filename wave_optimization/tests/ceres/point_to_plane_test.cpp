#include <ceres/gradient_checker.h>

#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/point_to_plane_interpolated_transform.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"

namespace wave {

TEST(Residual_test, SE3pointToPlaneAnalytic) {
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
    jacobian[0] = new double[12];

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, 0};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};
    double scale = 0.6;
    double residual = 0;

    ceres::CostFunction* cost_function = new SE3PointToPlane(pt, ptA, ptB, ptC, &scale, 1.0);
    cost_function->Evaluate(trans, &residual, jacobian);

    EXPECT_NEAR(residual, -4, 1e-4);

    ceres::LocalParameterization *se3_param = new SE3Parameterization;
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3_param);

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_TRUE(g_check.Probe(trans, 1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

}
