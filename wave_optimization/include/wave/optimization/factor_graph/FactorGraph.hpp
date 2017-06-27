/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/Factor.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/ceres/CeresOptimizer.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * A graph representing connections between measurements and unknown variables
 *
 * The FactorGraph uses an optimizer backend. Currently the only optimizer
 * available is Ceres.
 */
class FactorGraph {
 public:
    // Types
    using size_type = std::size_t;
    using iterator = std::vector<std::shared_ptr<FactorBase>>::iterator;
    using const_iterator =
      std::vector<std::shared_ptr<FactorBase>>::const_iterator;

    // Constructors
    FactorGraph();

    // Capacity

    /** Return the number of factors in the graph */
    size_type countFactors() const noexcept;

    bool empty() const noexcept;

    // Modifiers

    /** Add a factor with the given measurement function
     *
     * @param f The measurement function. See Factor::FuncType for the required
     * signature.
     * @param meas The FactorMeasurement stored in the factor. Its type must
     * match the measurement function signature.
     * @param variables Pointers to the FactorVariable%s used by the measurement
     * function. Their types must match the measurement function signature.
     */
    template <typename Functor,
              template <typename...> class M,
              template <typename...> class... V>
    void addFactor(const FactorMeasurement<M> &measurement,
                   std::shared_ptr<FactorVariable<V>>... variables);

    template <template <typename...> class M>
    void addPrior(const FactorMeasurement<M> &measurement,
                  std::shared_ptr<FactorVariable<M>> variable);

    // Note the first argument must be written as
    // `typename FactorVariable<V>::ValueType`, and not simply `V<double>`,
    // because of an esoteric name deduction problem. See
    // https://stackoverflow.com/questions/41642359
    // http://www.open-std.org/jtc1/sc22/wg21/docs/cwg_active.html#1979

    template <template <typename...> class V>
    void addPerfectPrior(
      const typename FactorVariable<V>::ValueType &measured_value,
      std::shared_ptr<FactorVariable<V>> variable);

    // Iterators

    /** Return iterator over factors */
    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    // Output

    friend std::ostream &operator<<(std::ostream &os, const FactorGraph &graph);

 private:
    std::vector<std::shared_ptr<FactorBase>> factors;
    Ceres
};

/** @} group optimization */
}  // namespace wave

#include "wave/optimization/factor_graph/impl/FactorGraph.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP
