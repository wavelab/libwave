#include <ceres/ceres.h>
#include <Eigen/Eigenvalues>

#include "wave/wave_test.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/trajectory_prior.hpp"
#include "wave/optimization/ceres/null_SE3_parameterization.hpp"

namespace wave {

/**
 * Test that a trajectory prior works correctly during optimization
 * Due to local parameterization hack, gradient checker does not work
 */
TEST(trajectory_prior, unit_weights) {
    Vec6 twist, prior_twist;
    twist << 0.1, -0.4, 0.3, 1, 10, -5;
    prior_twist << -0.3, 0.2, 0.5, -3, 2, -8;
    Transformation prior, initial;
    prior.setFromExpMap(twist);

    ceres::CostFunction* cost_function = new TrajectoryPrior(Eigen::Matrix<double, 12, 12>::Identity(), prior.inverse(), prior_twist);

    ceres::LocalParameterization *se3_param = new NullSE3Parameterization;

    ceres::Problem problem;
    problem.AddParameterBlock(initial.getInternalMatrix().data(), 12, se3_param);
    problem.AddParameterBlock(twist.data(), 6);

    problem.AddResidualBlock(cost_function, nullptr, initial.getInternalMatrix().data(), twist.data());

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport();

    auto diff = initial.manifoldMinus(prior);
    EXPECT_NEAR(diff.norm(), 0, 1e-8);
    EXPECT_NEAR((twist - prior_twist).norm(), 0, 1e-8);
}

}