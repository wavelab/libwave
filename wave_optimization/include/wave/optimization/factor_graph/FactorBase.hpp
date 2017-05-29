/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP

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
    virtual ~FactorBase() = default;
    /** Evaluate this factor to obtain residuals and jacobians.
     *
     * Types of `Factor` implement this function.
     *
     * This interface is based on Ceres' `CostFunction`. See
     * http://ceres-solver.org/nnls_modeling.html#costfunction
     */
    virtual bool evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) noexcept = 0;

    /** Print representation of the object for debugging.
     * Called by `operator<<`. */
    virtual void print(std::ostream &os) const = 0;
};


inline std::ostream &operator<<(std::ostream &os, const FactorBase &factor) {
    factor.print(os);
    return os;
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
