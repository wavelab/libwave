namespace wave {

template<typename F,
        template<typename...> class M,
        template<typename...> class... V>
void CeresOptimizer::addFactor(const Factor<F, M, V...> &factor) {
    // We make a vector of residual block pointers and pass it to
    // AddResidualBlock because Ceres' implementation forms a vector
    // anyway.
    auto data_ptrs = std::vector<double *>{};

    for (const auto &v : factor.variables()) {
        const auto &v_ptrs = v->blockData();
        const auto &v_sizes = v->blockSizes();
        data_ptrs.insert(data_ptrs.end(), v_ptrs.begin(), v_ptrs.end());

        for (auto i = 0u; i < v_ptrs.size(); ++i) {
            // Explicitly adding parameters "causes additional correctness
            // checking"
            // @todo can add local parametrization in this call
            this->problem.AddParameterBlock(v_ptrs[i], v_sizes[i]);

            // Set parameter blocks constant if the factor is a zero-noise prior
            if (factor->isPerfectPrior()) {
                this->problem.SetParameterBlockConstant(v_ptrs[i]);
            }
        }
    }

    // Finally, give ceres the cost function and its parameter blocks.
    if (!factor.isPerfectPrior()) {
        this->problem.AddResidualBlock(
                factor.costFunction().release(), nullptr, data_ptrs);
    }
}

void CeresOptimizer::evaluateGraph() {
    // Initialize the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    // Solve the problem, and write the estimated values to the variables
    // in the graph
    ceres::Solve(options, &this->problem, &summary);
}

}  // namespace wave
