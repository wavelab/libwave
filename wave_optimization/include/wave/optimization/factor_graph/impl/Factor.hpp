#include <ceres/autodiff_cost_function.h>

namespace wave {

template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
Factor<F, M, V...>::Factor(FactorMeasurement<M> measurement,
                           std::shared_ptr<FactorVariable<V>>... variable_ptrs)
    : measurement{measurement}, variable_ptrs{{variable_ptrs...}} {}

template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
template <typename T, typename O>
bool Factor<F, M, V...>::evaluate(const V<T, O> &... variables,
                                  M<T, O> &residuals) const noexcept {
    // Call the measurement function
    bool ok = F::template evaluate<T, O>(variables..., residuals);

    if (ok) {
        // Calculate the normalized residual
        const auto &L = this->measurement.noise.inverseSqrtCov();
        residuals = L * (residuals - this->measurement.value);
    }
    return ok;
};

template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
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
