#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <memory>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/factor.hpp"
#include "wave/optimization/factor_graph/variable.hpp"

namespace wave {

class FactorGraph {
 public:
    // Types
    using size_type = std::size_t;

    // Constructors
    FactorGraph();

    // Capacity

    /** Return number of variables in the graph */
    size_type countVariables() const noexcept;

    /** Return number of factors in the graph */
    size_type countFactors() const noexcept;

    /** Return total number of factors and variables */
    size_type size() const noexcept;

    bool empty() const noexcept;

    // Modifiers

    template <typename FactorType, typename... Args>
    void addFactor(Args &&... args);

    // Output

    friend std::ostream &operator<<(std::ostream &os, const FactorGraph &graph);

 private:
    std::vector<std::shared_ptr<FactorBase>> factors;
    std::multimap<VariableId, std::shared_ptr<FactorVariable>> variables;
};

}  // namespace wave

#include "impl/graph.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_GRAPH_HPP
