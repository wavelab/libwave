#include "wave/optimization/ceres/ceres_examples.hpp"

namespace wave {

void runAutoDiffExample(void) {
    // The variable to solve for with its initial value.
    double x = 5.0;
    double initial_x = x;

    // Build the problem.
    ceres::Problem problem;

    // Set up cost function (also known as residual)
    ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<AutoDiffCostFunctor, 1, 1>(
        new AutoDiffCostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
}

void runNumericalDiffExample(void) {
    // The variable to solve for with its initial value.
    double x = 5.0;
    double initial_x = x;

    // Build the problem.
    ceres::Problem problem;

    // Set up cost function (also known as residual)
    ceres::CostFunction *cost_function = new ceres::
      NumericDiffCostFunction<NumericalDiffCostFunctor, ceres::CENTRAL, 1, 1>(
        new NumericalDiffCostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
}

void runAnalyticalDiffExample(void) {
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // Build the problem.
    ceres::Problem problem;

    // Set up cost function (also known as residual).
    ceres::CostFunction *cost_function = new AnalyticalCostFunction;
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
}

}  // end of wave namespace
