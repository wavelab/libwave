#include "wave/utils/template_helpers.hpp"

namespace wave {

namespace internal {
// Template helpers used only in this file

/** Constructs a FactorValue mapping the given raw array.
 *
 * @tparam V the FactorValue template
 * @tparam T the scalar type used by the optimizer
 * @param param the pointer provided by the optimizer
 * @return a FactorValue mapping the given array
 */
template <template <typename...> class V, typename T>
inline V<T, FactorValueOptions::Map> make_composed_value(T const *const &ptr) {
    // The array is const, but ValueView maps non-const arrays. It would be
    // complicated to have two variants of ValueView, one for const and one
    // for non-const arrays. In this case, we know that the ValueView itself
    // is const, and it won't modify the array itself - thus, nothing will
    // modify the array, and it's "safe" to cast away the constness. Still,
    // @todo: reconsider this cast
    return V<T, FactorValueOptions::Map>{const_cast<T *>(ptr)};
}


template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
class FactorCostFunctor {
    constexpr static int N = sizeof...(V);

 public:
    template <typename Scalar>
    bool operator()(tmp::replacet<Scalar, V> const *const... raw_params,
                    Scalar *raw_residuals) const {
        auto residuals = M<Scalar, FactorValueOptions::Map>{raw_residuals};
        return this->factor->template evaluate<Scalar, FactorValueOptions::Map>(
          make_composed_value<V>(raw_params)..., residuals);
    }

    std::shared_ptr<Factor<F, M, V...>> factor;
};

template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
ceres::CostFunction *costFunctionForFactor(
  std::shared_ptr<Factor<F, M, V...>> factor) {
    using Functor = FactorCostFunctor<F, M, V...>;

    using CostFunction =
      ceres::AutoDiffCostFunction<Functor, M<double>::Size, V<double>::Size...>;

    return new CostFunction{new Functor{std::move(factor)}};
}

}  // namespace internal


template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
void CeresOptimizer::addFactor(std::shared_ptr<Factor<F, M, V...>> factor) {
    // We make a vector of residual block pointers and pass it to
    // AddResidualBlock because Ceres' implementation forms a vector
    // anyway.
    auto data_ptrs = std::vector<double *>{};

    for (const auto &v : factor->variables()) {
        data_ptrs.push_back(v->data());

        // Explicitly adding parameters "causes additional correctness
        // checking"
        // @todo can add local parametrization in this call
        this->problem.AddParameterBlock(v->data(), v->size());
    }

    // Finally, give ceres the cost function and its parameter blocks.
    this->problem.AddResidualBlock(
      internal::costFunctionForFactor(std::move(factor)), nullptr, data_ptrs);
}

template <template <typename...> class V>
inline void CeresOptimizer::addPerfectPrior(
  std::shared_ptr<PerfectPrior<V>> factor) {
    // Although a pefect prior acts on only one variable, it may be composed of
    // multiple FactorValues. Get all the data pointers.
    const auto &v = factor->variables().front();
    const auto ptr = v->data();
    // Add parameter block to the problem, and set constant since this is
    // a zero-noise prior
    this->problem.AddParameterBlock(ptr, v->size());

    // First, check that the block is not already constant - if so, there's
    // a conflict
    if (this->problem.IsParameterBlockConstant(ptr)) {
        throw std::logic_error(
          "Parameter block of ceres problem is already marked constant. "
          "Probably, multiple perfect priors have been added to the same "
          "variables");
    }

    this->problem.SetParameterBlockConstant(ptr);
}

inline void CeresOptimizer::evaluateGraph() {
    // Initialize the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    // Solve the problem, and write the estimated values to the variables
    // in the graph
    ceres::Solve(options, &this->problem, &summary);
}

}  // namespace wave
