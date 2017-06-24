#include "wave/optimization/factor_graph/FactorMeasurement.hpp"
#include "wave/optimization/factor_graph/PerfectPrior.hpp"

namespace wave {

namespace internal {

/**
 * Trivial measurement function for a prior, f(X) = X
 */
template <typename V>
inline bool identityMeasurementFunction(const V &variable, V &result) {
    result = variable;
    return true;
}

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

// Modifiers

template <typename FuncType, typename MeasType, typename... VarTypes>
inline void FactorGraph::addFactor(FuncType f,
                                   const MeasType &measurement,
                                   std::shared_ptr<VarTypes>... variables) {
    using FactorType = Factor<MeasType, VarTypes...>;

    // Give a nice error message if the function type is wrong
    static_assert(
      std::is_same<typename FactorType::FuncType *, FuncType>::value,
      "The given measurement function is of incorrect type");

    this->factors.emplace_back(
      std::make_shared<FactorType>(f, measurement, std::move(variables)...));
};

template <typename MeasType>
inline void FactorGraph::addPrior(
  const MeasType &measurement,
  std::shared_ptr<typename MeasType::VarType> variable) {
    return this->addFactor(
      internal::identityMeasurementFunction<typename MeasType::ViewType>,
      measurement,
      std::move(variable));
}

template <typename VarType>
inline void FactorGraph::addPerfectPrior(
  const typename VarType::MappedType &measured_value,
  std::shared_ptr<VarType> variable) {
    using MeasType = FactorMeasurement<typename VarType::ViewType, void>;

    this->factors.emplace_back(std::make_shared<PerfectPrior<VarType>>(
      MeasType{measured_value}, std::move(variable)));
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
