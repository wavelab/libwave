#include <ceres/autodiff_cost_function.h>
#include "wave/optimization/factor_graph/template_helpers.hpp"

namespace wave {

namespace internal {
// Template helpers used only in this file

/** Constructs a ValueView mapping the given raw array.
 *
 * @tparam V the type of FactorVariable
 * @tparam T the scalar type used by the optimizer
 * @param param the pointer provided by the optimizer
 * @return a FactorValue mapping the given array
 */
template <typename Vv, typename T>
inline Vv make_value(T const *const param) {
    // The array is const, but ValueView maps non-const arrays. It would be
    // complicated to have two variants of ValueView, one for const and one
    // for non-const arrays. In this case, we know that the ValueView itself
    // is const, and it won't modify the array itself - thus, nothing will
    // modify the array, and it's "safe" to cast away the constness. Still,
    // @todo: reconsider this cast
    const auto ptr = const_cast<T *>(param);
    return Vv{ptr};
}

template <typename F, typename ValueTuple, typename T>
struct FunctorHelper;

template <typename F, typename Mv, typename... Vv, typename T>
struct FunctorHelper<F, std::tuple<Mv, Vv...>, T> {
    bool operator()(tmp::replace<T, Vv> const *const... params, Mv& residuals) {
        return f(make_value<Vv, T>(params)..., residuals);
    }
    F f{};
};


template <typename Functor, typename SizeSeq>
struct get_cost_function;

template <typename Functor, int... ValueSizes>
struct get_cost_function<Functor, tmp::index_sequence<ValueSizes...>> {
    using type = ceres::AutoDiffCostFunction<Functor, ValueSizes...>;
};

}  // namespace internal

template <typename F,
          template <typename> class M,
          template <typename> class... V>
Factor<F, M, V...>::Factor(FactorMeasurement<M> measurement,
                           std::shared_ptr<FactorVariable<V>>... variable_ptrs)
    : measurement{measurement}, variable_ptrs{{variable_ptrs...}} {}

template <typename F,
          template <typename> class M,
          template <typename> class... V>
std::unique_ptr<ceres::CostFunction> Factor<F, M, V...>::costFunction() const
  noexcept {
    using Functor = FactorCostFunctor<F, M, V...>;
    using CostFunction = internal::get_cost_function<
      F,
      internal::get_value_sizes<V<double>...>>;
    return std::unique_ptr<ceres::CostFunction>{
      new CostFunction{new Functor{this}}};
}

template <typename F,
          template <typename> class M,
          template <typename> class... V>
template <typename... T>
bool Factor<F, M, V...>::evaluateRaw(
  T const *const... parameters, T *raw_residuals) const
  noexcept {
    auto residuals = internal::make_value<M<Map<T>>, T>(raw_residuals);
    // Call the measurement function
    bool ok = internal::FunctorHelper<F, internal::get_value_types<M<Map<T>>, V<Map<T>>...>, T>{}(parameters..., residuals);
    if (ok) {
        // Calculate the normalized residual
        const auto &L = measurement.noise.inverseSqrtCov();
        residuals = L * (residuals - measurement.value);
    }
    return ok;
}

template <typename F,
          template <typename> class M,
          template <typename> class... V>
void Factor<F, M, V...>::print(std::ostream &os) const {
    os << "[";
    os << "Factor arity " << NumVars << ", ";
    os << "variables: ";
    for (auto i = 0; i < NumVars; ++i) {
        const auto &v = this->variable_ptrs[i];
        os << *v << "(" << v << ")";
        if (i < NumVars - 1) {
            os << ", ";
        }
    }
    os << "]";
}

}  // namespace wave
