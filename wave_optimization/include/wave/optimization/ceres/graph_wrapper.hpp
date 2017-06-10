/**
 * @file
 * @ingroup optimization
 * Ceres wrappers for FactorGraph classes
 *
 * @todo reorganize - move to another file
 * @todo build up ceres Problem simultaneously while building up FactorGraph
 */
#ifndef WAVE_OPTIMIZATION_CERES_GRAPH_WRAPPER_HPP
#define WAVE_OPTIMIZATION_CERES_GRAPH_WRAPPER_HPP

#include <ceres/ceres.h>
#include <functional>
#include "wave/optimization/factor_graph/FactorGraph.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

void addFactorToProblem(ceres::Problem &problem, FactorBase &factor) {
    // We make a vector of residual block pointers and pass it to
    // AddResidualBlock because Ceres' implementation forms a vector
    // anyway.
    auto data_ptrs = std::vector<double *>{};

    for (const auto &v : factor.variables()) {
        data_ptrs.push_back(v->data());

        // Explicitly adding parameters "causes additional correctness
        // checking"
        // @todo can add local parametrization in this call
        problem.AddParameterBlock(v->data(), v->size());

        // Set parameter blocks constant if the variable is so marked
        if (v->isFixed()) {
            problem.SetParameterBlockConstant(v->data());
        }
    }

    // Give ceres the cost function and its parameter blocks.
    problem.AddResidualBlock(
      factor.costFunction().release(), nullptr, data_ptrs);
}

/**
 * Evaluates the factors in graph. Sets the variable to the estimated values.
 *
 * @todo change interface to expose results / ceres summary output
 */
void evaluateGraph(FactorGraph &graph) {
    // Initialize the problem
    ceres::Problem problem;

    for (const auto &ptr : graph) {
        addFactorToProblem(problem, *ptr);
    }

    // Initialize the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    // Solve the problem, and write the estimated values to the variables
    // in the graph
    ceres::Solve(options, &problem, &summary);
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_CERES_GRAPH_WRAPPER_HPP
