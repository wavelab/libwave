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

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * A graph representing connections between measurements and unknown variables
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
    template <typename FuncType, typename MeasType, typename... VarTypes>
    void addFactor(FuncType f,
                   const MeasType &measurement,
                   std::shared_ptr<VarTypes>... variables);

    template <typename MeasType>
    void addPrior(const MeasType &measurement,
                  std::shared_ptr<typename MeasType::VarType> variable);

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
};

/** @} group optimization */
}  // namespace wave

#include "wave/optimization/factor_graph/impl/FactorGraph.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP
