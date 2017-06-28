/**
 * @file
 * @ingroup optimization
 * Ceres wrappers for FactorGraph classes
 *
 * @todo reorganize - move to another file
 * @todo build up ceres Problem simultaneously while building up FactorGraph
 */
#ifndef WAVE_OPTIMIZATION_CERES_CERESOPTIMIZER_HPP
#define WAVE_OPTIMIZATION_CERES_CERESOPTIMIZER_HPP

#include <ceres/ceres.h>
#include <functional>
#include "wave/optimization/factor_graph/Factor.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Backend for FactorGraph providing an interface to ceres-solver
 */
class CeresOptimizer {
 public:
    /** Add a factor of the corresponding type
     * @see Factor
     */
    template <typename F,
              template <typename...> class M,
              template <typename...> class... V>
    void addFactor(const Factor<F, M, V...> &factor);

    /** Actually optimize over the graph.
     *
     * Set each variable to the estimated values.
     *
     * @todo change interface to expose results / ceres summary output
     */
    void evaluateGraph();

 private:
    ceres::Problem problem;
};

/** @} group optimization */
}  // namespace wave

#include "impl/CeresOptimizer.hpp"

#endif  // WAVE_OPTIMIZATION_CERES_CERESOPTIMIZER_HPP
