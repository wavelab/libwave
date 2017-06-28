#include "wave/optimization/factor_graph/FactorMeasurement.hpp"
#include "wave/optimization/factor_graph/PerfectPrior.hpp"

namespace wave {

namespace internal {

/**
 * Trivial measurement function for a prior, f(X) = X
 */
template <template <typename...> class V>
struct IdentityMeasurementFunctor {
    template <typename T, typename O = void>
    static bool evaluate(const V<T, O> &variable, V<T, O> &result) {
        result = variable;
        return true;
    }
};


}  // namespace internal

// Constructors
inline FactorGraph::FactorGraph() {}

// Capacity

inline FactorGraph::size_type FactorGraph::countFactors() const noexcept {
    return this->factors.size();
}

inline bool FactorGraph::empty() const noexcept {
    return this->factors.empty();
}
template <typename T>
struct Debug;

// Modifiers
template <typename Functor,
          template <typename...> class M,
          template <typename...> class... V>
inline void FactorGraph::addFactor(
  const FactorMeasurement<M> &measurement,
  std::shared_ptr<FactorVariable<V>>... variables) {
    using FactorType = Factor<Functor, M, V...>;

    // Give a nice error message immediately if the function type is wrong
    using MemberFuncType = decltype(&FactorType::template evaluate<double>);
    using ExpectedFuncType = typename tmp::clean_method<MemberFuncType>::type;
    using ActualFuncType = decltype(Functor::template evaluate<double>);
    static_assert(std::is_same<ExpectedFuncType, ActualFuncType>::value,
                  "The given measurement function is of incorrect type");

    auto factor =
      std::make_shared<FactorType>(measurement, std::move(variables)...);

    this->factors.push_back(factor);

    // Add to the back-end optimizer
    optimizer.addFactor(factor);
};


template <template <typename...> class M>
inline void FactorGraph::addPrior(const FactorMeasurement<M> &measurement,
                                  std::shared_ptr<FactorVariable<M>> variable) {
    return this->addFactor<internal::IdentityMeasurementFunctor<M>>(
      measurement, std::move(variable));
}

template <template <typename...> class V>
inline void FactorGraph::addPerfectPrior(
  const typename FactorVariable<V>::ValueType &measured_value,
  std::shared_ptr<FactorVariable<V>> variable) {
    using MeasType = FactorMeasurement<V, void>;

    this->factors.emplace_back(std::make_shared<PerfectPrior<V>>(
      MeasType{measured_value}, std::move(variable)));
}

inline void FactorGraph::evaluate() {
    this->optimizer.evaluateGraph();
}


// Iterators

inline typename FactorGraph::iterator FactorGraph::begin() noexcept {
    return this->factors.begin();
}

inline typename FactorGraph::iterator FactorGraph::end() noexcept {
    return this->factors.end();
}

inline typename FactorGraph::const_iterator FactorGraph::begin() const
  noexcept {
    return this->factors.begin();
}

inline typename FactorGraph::const_iterator FactorGraph::end() const noexcept {
    return this->factors.end();
}

inline std::ostream &operator<<(std::ostream &os, const FactorGraph &graph) {
    os << "FactorGraph " << graph.countFactors() << " factors [";
    const auto N = graph.countFactors();
    for (auto i = 0 * N; i < N; ++i) {
        os << *graph.factors[i];
        if (i < N - 1) {
            os << ", ";
        }
    }
    os << "]";

    return os;
}

}  // namespace wave
