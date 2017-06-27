/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP

#include <ceres/cost_function.h>
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Abstract base for all factor types.
 *
 * Do not derive this class directly, instead, derive a specialization of
 * `Factor`.
 */
class FactorBase {
 public:
    using VarVectorType =
      const std::vector<std::shared_ptr<FactorVariableBase>>;
    using const_iterator = VarVectorType::const_iterator;

    virtual ~FactorBase() = default;

    /** Return a Ceres cost function object which computes the residuals of
     * this factor.
     */
    virtual std::unique_ptr<ceres::CostFunction> costFunction() const
      noexcept = 0;

    /** Print representation of the object for debugging.
     * Called by `operator<<`. */
    virtual void print(std::ostream &os) const = 0;

    /** The arity (number of variables connected) of the factor */
    virtual int size() const = 0;

    /** Get a reference to the vector of variable pointers */
    virtual const VarVectorType &variables() const noexcept = 0;

    /** Rturn true if this factor is a zero-noise prior */
    virtual bool isPerfectPrior() const noexcept = 0;
};


inline std::ostream &operator<<(std::ostream &os, const FactorBase &factor) {
    factor.print(os);
    return os;
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
