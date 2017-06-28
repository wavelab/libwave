/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"
#include "wave/optimization/factor_graph/FactorValue.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/OutputMap.hpp"
#include "wave/utils/template_helpers.hpp"
#include "FactorMeasurement.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Template for factors of different variable types.
 *
 * The residuals in a least squares function can be described as
 * @f[
 * r(\Theta) = f(\Theta) - Z
 * @f]
 *
 * where
 *
 * - @f$ Z @f$ is the actual measurement
 * - @f$ \Theta @f$ is one or more variables
 * - @f$ f(\Theta) @f$ is the measurement function, producing an estimate of @f$
 * Z @f$ given some variables @f$ \Theta @f$
 *
 * These three things are stored in a Factor. Most of the work in defining a
 * factor is defining a measurement function.
 * See Factor::FuncType for the required function signature.
 *
 * It is typically unnecessary to directly instantiate or derive this template;
 * instead, use FactorGraph::addFactor.
 *
 * @tparam MeasType The type of measurement used by this factor
 * @tparam VarTypes The list of variables types operated on by this factor
 */
template <typename F,
          template <typename...> class M,
          template <typename...> class... V>
class Factor : public FactorBase {
    using FactorType = Factor<F, M, V...>;

 public:
    constexpr static int NumVars = sizeof...(V);
    using VarArrayType = FactorBase::VarVectorType;

    /** The deduced type the used-defined measurement function must have.
     *
     * The user-defined measurement function must have the following signature:
     *
     * @param[in] variables each variable this function operates on, passed in
     * as the correspoing `ValueView` object
     * @param[out] residuals residuals calculated using the given variables
     * @param[out] jacobians
     * @parblock
     * Jacobian matrices with respect to each variable. There is one jacobian
     * parameter for each variable parameter, in the same order.
     * @endparblock
     *
     * @note  Each residual and jacobian output parameter is a special map
     * object which may evaluate to false to indicate the jacobian should not be
     * calculated on this call. Wrap calculation and assignment of each
     * parameter `j` in a conditional: `if (j) {...}`.
     *
     * @return true if calculation was successful, false on error
     */
    template <typename T, typename O = void>
    using FuncType = bool (*)(const V<T, O> &..., M<T, O> &);

    using const_iterator = typename VarArrayType::const_iterator;

    /** Construct with the given measurement and variables. */
    explicit Factor(FactorMeasurement<M> measurement,
                    std::shared_ptr<FactorVariable<V>>... variable_ptrs);

    int size() const override {
        return NumVars;
    }

    template <typename T, typename O = void>
    bool evaluate(const V<T, O> &... variables, M<T, O> &residuals) const
      noexcept;

    /** Get a reference to the vector of variable pointers */
    const VarVectorType &variables() const noexcept override {
        return this->variable_ptrs;
    }

    /** Return true if this factor is a zero-noise prior */
    bool isPerfectPrior() const noexcept override {
        return false;
    }

    /** Print a representation for debugging. Used by operator<< */
    void print(std::ostream &os) const override;

 private:
    /** Storage of the measurement */
    FactorMeasurement<M> measurement;

    /** Pointers to the variables this factor is linked to */
    VarArrayType variable_ptrs;
};

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
