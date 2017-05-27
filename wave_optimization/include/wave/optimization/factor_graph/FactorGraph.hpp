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

    /** Return number of factors in the graph */
    size_type countFactors() const noexcept;

    bool empty() const noexcept;

    // Modifiers

    template <typename FactorType, typename... Args>
    void addFactor(Args &&... args);

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
