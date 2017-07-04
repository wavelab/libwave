#include "wave/utils/template_helpers.hpp"

namespace wave {

namespace internal {
// Template helpers used only in this file

/** Define an index_sequence of nested FactorValue sizes */
template <template <typename...> class... V>
using get_value_sizes = typename tmp::concat_index_sequence<
  typename internal::factor_value_traits<V<double>>::BlockSizes...>::type;

/** Concatenate the FactorValue types of each variable in a Factor into one
 * std::tuple.
 */
template <template <typename...> class... V>
using expand_value_tmpls = typename tmp::tuple_cat_result<
  typename internal::factor_value_traits<V<double>>::ValueTuple...>;

/**
 * Define a type_sequence of index_sequences mapping nested values in a factor
 * to entries in a flattened parameter list (as used by Ceres).
 *
 * For example, consider a factor of these three variables:
 * 1. a simple variable with one value
 * 2. a composed variable with four nested values
 * 3. a composed variable with two nested values
 *
 * (note the flattened parameter list would have seven entries)
 *
 * The instantiation `get_value_indices<0, index_sequence<1, 4, 2>>` will
 * produce a sequence `type_sequence<index_sequence<0>,
 * index_sequence<1, 2, 3, 4>, index_sequence<5, 6>>` - meaning the 0th entry
 * in the flattened list corresponds to the single value in the 0th variable,
 * the 1st to 4th entries in the list corresponding the to 0th to 3rd entries in
 * the 1st variable, etc.
 *
 * @tparam S the starting index (used for recursion; pass 0)
 * @tparam ISeq the index_sequence of value sizes
 * @tparam OutSeq used to build up the output; pass an empty sequence
 */
template <int S, typename ISeq, typename OutSeq = tmp::type_sequence<>>
struct get_value_indices;

// Recursively build output, consuming the first element in the input sequence
template <int S, int Head, int... Tail, typename... Out>
struct get_value_indices<S,
                         tmp::index_sequence<Head, Tail...>,
                         tmp::type_sequence<Out...>>
  : get_value_indices<
      S + Head,
      tmp::index_sequence<Tail...>,
      tmp::type_sequence<Out...,
                         typename tmp::make_index_sequence<Head, S>::type>> {};

// Base case: only one element in input sequence
template <int S, int Head, typename... Out>
struct get_value_indices<S,
                         tmp::index_sequence<Head>,
                         tmp::type_sequence<Out...>>
  : tmp::type_sequence<Out...,
                       typename tmp::make_index_sequence<Head, S>::type> {};

/**
 * Instantiates `get_value_indices` with the appropriate arguments for the given
 * value types.
 */
template <template <typename...> class... V>
using get_value_indices_t = typename get_value_indices<
  0,
  typename tmp::index_sequence<
    internal::factor_value_traits<V<double>>::NumValues...>>::type;

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
                                  internal::get_value_indices_t<V...>{},
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
