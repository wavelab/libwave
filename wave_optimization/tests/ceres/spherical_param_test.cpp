#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/optimization/ceres/local_params/spherical_parameterization.hpp"
#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace {

class TestSpherical : public ceres::SizedCostFunction<3, 3> {
 private:
    const double *const normal;

 public:
    virtual ~TestSpherical() {};

    TestSpherical(const double *const normal) : normal(normal) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        // Want to align the optimization parameter with the normal, error is a - b

        residuals[0] = -normal[0] + parameters[0][0];
        residuals[1] = -normal[1] + parameters[0][1];
        residuals[2] = -normal[2] + parameters[0][2];

        if (jacobians) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobians[0]);
            J.setIdentity();
        }
        return true;
    }
};

}

namespace wave {

TEST(easy_cost_function, alignment) {
    Vec3 normal;
    normal << -1, 2, 4;
    normal.normalize();

    Vec3 initial;
    initial.setRandom();
    initial.normalize();

    ceres::Problem problem;
    ceres::CostFunction *cost = new TestSpherical(normal.data());
    problem.AddResidualBlock(cost, nullptr, initial.data());

    ceres::LocalParameterization *spherical = new SphericalParameterization();
    problem.SetParameterization(initial.data(), spherical);

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport();

    EXPECT_LT((initial - normal).norm(), 1e-6);
    EXPECT_NEAR(initial.norm(), 1.0, 1e-6);
}

}