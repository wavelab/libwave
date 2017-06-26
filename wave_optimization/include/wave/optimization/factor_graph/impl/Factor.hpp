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

//template <typename T, typename VSeq, typename ISeq>
//struct make_composed_value;

template <typename T, template <typename> class V, int N, int... Is>
inline V<Map<T>> make_composed_value(std::array<T const * const, N> ptrs,  tmp::index_sequence<Is...>) {
    return V<Map<T>>{ptrs[Is]...};
};

//template <template <typename> class... Vv, int... Is>
//struct collapse<tmp::tmpl_sequence<Vv...>, tmp::index_sequence<Is...>>
//{
//    using type = tmp::tmpl_sequence<get_value_indices
//};

template <typename F, template <typename> class M, typename TmplTuple, template <typename> class... V>
struct FactorCostFunctor;

template <typename F,
          template <typename> class M,
          template <typename> class... Vv,
          template <typename> class... V>
struct FactorCostFunctor<F, M, tmp::tmpl_sequence<Vv...>, V...> {
    template <typename T>
    bool operator()(tmp::replacet<T, Vv> const *const... params,
                    T *raw_residuals) const {
        auto residuals = M<Map<T>>{raw_residuals};

        // Call the measurement function
        bool ok = F{}(internal::make_composed_value<T, V, sizeof...(V), internal::get_value_indices<V>>(params)..., residuals);
        if (ok) {
            // Calculate the normalized residual
            const auto &L = this->meas.noise.inverseSqrtCov();
            residuals = L * (residuals - this->meas.value);
        }
        return ok;
    }

    const FactorMeasurement<M> &meas;
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
    using Functor =
      internal::FactorCostFunctor<F, M, internal::expand_value_tmpls<V...>, V...>;
    using CostFunction = typename internal::
      get_cost_function<Functor, internal::get_value_sizes<M, V...>>::type;
    return std::unique_ptr<ceres::CostFunction>{
      new CostFunction{new Functor{this->measurement}}};
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
