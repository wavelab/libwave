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

/**
 * Ceres cost function wrapping `Factor::evaluateRaw`.
 */
class FactorCostFunction : public ceres::CostFunction {
 public:
    explicit FactorCostFunction(std::shared_ptr<FactorBase> factor)
        : factor{factor} {
        this->set_num_residuals(factor->residualSize());
        for (const auto &var : factor->variables()) {
            this->mutable_parameter_block_sizes()->push_back(var->size());
        }
    }

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override {
        return factor->evaluateRaw(parameters, residuals, jacobians);
    }

 private:
    std::shared_ptr<FactorBase> factor;
};

void addFactorToProblem(ceres::Problem &problem,
                        std::shared_ptr<FactorBase> factor) {
    // We make a vector of residual block pointers and pass it to
    // AddResidualBlock because Ceres' implementation forms a vector
    // anyway.
    auto data_ptrs = std::vector<double *>{};

    for (const auto &v : factor->variables()) {
        data_ptrs.push_back(v->data());

        // Explicitly adding parameters "causes additional correctness
        // checking"
        // @todo can add local parametrization in this call
        problem.AddParameterBlock(v->data(), v->size());

        // Set parameter blocks constant if the factor is a zero-noise prior
        if (factor->isPerfectPrior()) {
            problem.SetParameterBlockConstant(v->data());
        }
    }

    // Finally, give ceres the cost function and its parameter blocks.
    if (!factor->isPerfectPrior()) {
        problem.AddResidualBlock(
          new FactorCostFunction{std::move(factor)}, nullptr, data_ptrs);
    }
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
        addFactorToProblem(problem, ptr);
    }

    // Initialize the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    // Solve the problem, and write the estimated values to the variables
    // in the graph
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_CERES_GRAPH_WRAPPER_HPP
