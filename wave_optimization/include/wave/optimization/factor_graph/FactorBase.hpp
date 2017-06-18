/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP

#include <ceres/cost_function.h>
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

    /** Evaluate this factor to obtain residuals and jacobians.
     *
     * This function is not meant to be called by users. It is called by the
     * back-end optimizer which works with pointers to pointers of C-style
     * arrays. For clarity and type safety, it would be preferable `Factor`
     * class
     * implemented this method, providing the needed conversions.
     *
     * This interface is based on Ceres' `CostFunction`. See
     * http://ceres-solver.org/nnls_modeling.htmlmarch#costfunction
     */
    virtual bool evaluateRaw(double const *const *parameters,
                             double *residuals,
                             double **jacobians) const noexcept = 0;

    /** Print representation of the object for debugging.
     * Called by `operator<<`. */
    virtual void print(std::ostream &os) const = 0;

    virtual int size() const = 0;
    virtual int numResiduals() const = 0;
    virtual std::unique_ptr<ceres::CostFunction> costFunction() = 0;

    /** Get a reference to the vector of variable pointers */
    virtual const VarVectorType &variables() const noexcept = 0;

    /** Return true if this factor is a zero-noise prior */
    virtual bool isPerfectPrior() const noexcept = 0;
};


inline std::ostream &operator<<(std::ostream &os, const FactorBase &factor) {
    factor.print(os);
    return os;
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORBASE_HPP
