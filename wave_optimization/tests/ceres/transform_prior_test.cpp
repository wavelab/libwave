#include <ceres/gradient_checker.h>
#include <Eigen/Eigenvalues>

#include "wave/wave_test.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/optimization/ceres/transform_prior.hpp"
#include "wave/optimization/ceres/local_params/null_SE3_parameterization.hpp"

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

    Transformation<> val, saved_val;
    val.storage = Eigen::Map<const Mat34>(trans[0], 3, 4);
    saved_val = val;

    Vec6 error;
    Transformation<> prior_val;

    Eigen::Matrix<double, 6, 12, Eigen::RowMajor> analytical_jacobian, numerical_jacobian;
    numerical_jacobian.setZero();

    double *jacobian[1];
    jacobian[0] = analytical_jacobian.data();

    double *params[1];
    params[0] = val.storage.data();

    ceres::CostFunction* cost_function = new TransformPrior(Mat6::Identity(), prior_val);
    cost_function->Evaluate(params, error.data(), jacobian);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec6 perturb;
    Vec6 perturbed_error;
    for (uint32_t i = 0; i < 6; ++i) {
        perturb.setZero();
        perturb(i) = step;
        val = saved_val;
        val.manifoldPlus(perturb);

        cost_function->Evaluate(params, perturbed_error.data(), nullptr);
        numerical_jacobian.col(i) = (perturbed_error - error) / step;
    }

    MatX diff = analytical_jacobian - numerical_jacobian;
    EXPECT_NEAR(diff.norm(), 0, 1e-6);
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

    Transformation<> val, saved_val;
    val.storage = Eigen::Map<const Mat34>(trans[0], 3, 4);
    saved_val = val;

    Vec6 error;
    Transformation<> prior_val;

    Eigen::Matrix<double, 6, 12, Eigen::RowMajor> analytical_jacobian, numerical_jacobian;
    numerical_jacobian.setZero();

    double *jacobian[1];
    jacobian[0] = analytical_jacobian.data();

    double *params[1];
    params[0] = val.storage.data();

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
    cost_function->Evaluate(params, error.data(), jacobian);

    double step = std::sqrt(std::numeric_limits<double>::epsilon());
    Vec6 perturb;
    Vec6 perturbed_error;
    for (uint32_t i = 0; i < 6; ++i) {
        perturb.setZero();
        perturb(i) = step;
        val = saved_val;
        val.manifoldPlus(perturb);

        cost_function->Evaluate(params, perturbed_error.data(), nullptr);
        numerical_jacobian.col(i) = (perturbed_error - error) / step;
    }

    MatX diff = analytical_jacobian - numerical_jacobian;
    EXPECT_NEAR(diff.norm(), 0, 1e-6);
}

}