// This is to check whether I can sidestep having to provide a Jacobian in terms of the overparameterized version

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/optimization/ceres/local_params/null_SE3_parameterization.hpp"
#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace {

class TestPointToPoint : public ceres::SizedCostFunction<3, 12> {
 private:
    const double *const P1;
    const double *const P2;

 public:
    virtual ~TestPointToPoint(){};
    TestPointToPoint(const double *const p1, const double *const p2) : P1(p1), P2(p2) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        // modelled as P1 = T*P2
        // r = T*P2 - P1;
        // parameters in order:
        // R11 R21 R31 R12 R22 R32 R13 R23 R33 X Y Z
        double Tp[3] = {parameters[0][0] * this->P2[0] + parameters[0][3] * this->P2[1] + parameters[0][6] * this->P2[2] + parameters[0][9],
                        parameters[0][1] * this->P2[0] + parameters[0][4] * this->P2[1] + parameters[0][7] * this->P2[2] + parameters[0][10],
                        parameters[0][2] * this->P2[0] + parameters[0][5] * this->P2[1] + parameters[0][8] * this->P2[2] + parameters[0][11]};

        residuals[0] = Tp[0] - this->P1[0];
        residuals[1] = Tp[1] - this->P1[1];
        residuals[2] = Tp[2] - this->P1[2];

        if(jacobians) {
            Eigen::Matrix<double, 3, 6> local_jacobian;
            local_jacobian.setZero();
            local_jacobian.block<3,3>(0,3).setIdentity();
            local_jacobian(0,1) = Tp[2];
            local_jacobian(0,2) = -Tp[1];
            local_jacobian(1,0) = -Tp[2];
            local_jacobian(1,2) = Tp[0];
            local_jacobian(2,0) = Tp[1];
            local_jacobian(2,1) = -Tp[0];

            Eigen::Matrix<double, 6, 12> bogus_inflation;
            bogus_inflation.setZero();
            bogus_inflation.block<6,6>(0,0).setIdentity();

            Eigen::Matrix<double, 3, 12> bogus_jacobian = local_jacobian * bogus_inflation;

            Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>>(jacobians[0], 3, 12) = bogus_jacobian;
        }
        return true;
    }
};

}

namespace wave{

TEST(local_jacobians, fake_lift_jacobian) {
    Vec6 transformation_twist_parameters;
    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
            1.960945901104432, 3.037052911306709;
    Transformation<Eigen::Matrix<double, 3, 4>> exact_transform;
    exact_transform.setFromExpMap(transformation_twist_parameters);

    const int size = 3;
    Vec3 points[3];
    Vec3 points_transformed[3];
    points[0] = Vec3(0, 2, 0);
    points[1] = Vec3(40, 0, 0);
    points[2] = Vec3(40, 10, 0);

    double transform[12] = {0};
    transform[0] = 1;
    transform[4] = 1;
    transform[8] = 1;

    ceres::Problem problem;
    for (int i = 0; i < size; i++) {
        points_transformed[i] = exact_transform.transform(points[i]);
        ceres::CostFunction *cost_function = new TestPointToPoint(points_transformed[i].data(), points[i].data());
        problem.AddResidualBlock(cost_function, NULL, transform);
    }
    ceres::LocalParameterization *se3 = new NullSE3Parameterization();
    problem.SetParameterization(transform, se3);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport();

    Transformation<Eigen::Matrix<double, 3, 4>> solved_transform;
    Mat4 sol_matrix;

    sol_matrix << transform[0], transform[3], transform[6], transform[9],  //
            transform[1], transform[4], transform[7], transform[10],             //
            transform[2], transform[5], transform[8], transform[11],             //
            0, 0, 0, 1;                                                          //

    Mat4 errmat = sol_matrix - exact_transform.getMatrix();
    ASSERT_LE(errmat.norm(), 1e-9);
}

/**
 * This test is to verify that this hack breaks the gradient checker in Ceres
 * FYI, hack breaks the gradient checker in Ceres
 */
TEST(local_jacobians, gradient_checker) {
    Vec6 transformation_twist_parameters;
    transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
            1.960945901104432, 3.037052911306709;
    Transformation<Eigen::Matrix<double, 3, 4>> exact_transform;
    exact_transform.setFromExpMap(transformation_twist_parameters);

    Vec3 point;
    Vec3 points_transformed;
    point = Vec3(5, 2, 0);

    points_transformed = exact_transform.transform(point);
    ceres::CostFunction *cost_function = new TestPointToPoint(points_transformed.data(), point.data());

    ceres::LocalParameterization *se3 = new NullSE3Parameterization();
    std::vector<const ceres::LocalParameterization*> local_param_vec;
    local_param_vec.emplace_back(se3);

    const double **parameters;
    parameters = new const double *[1];
    parameters[0] = exact_transform.storage.data();

    ceres::NumericDiffOptions ndiff_options;
    ceres::GradientChecker g_check(cost_function, &local_param_vec, ndiff_options);
    ceres::GradientChecker::ProbeResults g_results;
    EXPECT_FALSE(g_check.Probe(parameters, 1e-6, &g_results));
    LOG_INFO("%s", g_results.error_log.c_str());
}

}
