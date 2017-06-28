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
template <typename T, template <typename...> class V, int N, int... Is>
inline V<T, FactorValueOptions::Map> make_composed_value(
  const std::array<T const *const, N> &ptrs, tmp::index_sequence<Is...>) {
    // The array is const, but ValueView maps non-const arrays. It would be
    // complicated to have two variants of ValueView, one for const and one
    // for non-const arrays. In this case, we know that the ValueView itself
    // is const, and it won't modify the array itself - thus, nothing will
    // modify the array, and it's "safe" to cast away the constness. Still,
    // @todo: reconsider this cast
    return V<T, FactorValueOptions::Map>{const_cast<T *>(ptrs[Is])...};
}

template <typename T,
          template <typename...> class V,
          int N,
          typename ISeq,
          int I>
inline V<T, FactorValueOptions::Map> make_composed_value1(
  const std::array<T const *const, N> &ptrs) {
    return make_composed_value<T, V, N>(ptrs, ISeq{});
}


template <typename F,
          template <typename...> class M,
          typename VTuple,
          template <typename...> class... V>
class FactorCostFunctor;

template <typename F,
          template <typename...> class M,
          typename... Vv,
          template <typename...> class... V>
class FactorCostFunctor<F, M, std::tuple<Vv...>, V...> {
    constexpr static int N = sizeof...(Vv);

 public:
    template <typename T>
    bool operator()(tmp::replace<T, Vv> const *const... raw_params,
                    T *raw_residuals) const {
        auto residuals = M<T, FactorValueOptions::Map>{raw_residuals};
        const auto &params = std::array<T const *const, N>{{raw_params...}};
        return this->callEvaluate(params,
                                  residuals,
                                  internal::get_value_indices<V...>{},
                                  tmp::make_index_sequence<sizeof...(V)>{});
    }

 private:
    template <typename T, typename... ISeq, int... Is>
    bool callEvaluate(const std::array<T const *const, N> &ptrs,
                      M<T, FactorValueOptions::Map> &residuals,
                      tmp::type_sequence<ISeq...> &&,
                      tmp::index_sequence<Is...> &&) const noexcept {
        return factor->template evaluate<T, FactorValueOptions::Map>(
          make_composed_value1<T, V, N, ISeq, Is>(ptrs)..., residuals);
    }

 public:
    std::shared_ptr<Factor<F, M, V...>> factor;
};

template <typename Functor, typename SizeSeq>
struct CostFunctionAlias;

template <typename Functor, int... ValueSizes>
struct CostFunctionAlias<Functor, tmp::index_sequence<ValueSizes...>> {
    using type = ceres::AutoDiffCostFunction<Functor, ValueSizes...>;
};

template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
ceres::CostFunction *costFunctionForFactor(
  std::shared_ptr<Factor<F, M, V...>> factor) {
    using Functor =
      FactorCostFunctor<F, M, internal::expand_value_tmpls<V...>, V...>;

    using CostFunction =
      typename CostFunctionAlias<Functor,
                                 internal::get_value_sizes<M, V...>>::type;

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
    if (!factor->isPerfectPrior()) {
        this->problem.AddResidualBlock(
          internal::costFunctionForFactor(std::move(factor)),
          nullptr,
          data_ptrs);
    }
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
