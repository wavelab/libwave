/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <boost/fusion/include/vector.hpp>
#include <memory>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"
#include "wave/optimization/factor_graph/OutputMap.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Abstract base for factors of different variable types.
 *
 * Any factor stored in a FactorGraph must define an `evaluate` function which
 * works with pointers to pointers of C-style arrays supplied by the optimizer.
 * For clarity and type safety, it would be preferable to work with
 * better-defined types such as fixed-size Eigen vectors, which requires
 * re-interpreting the arrays as Eigen types of the expected size. This template
 * does that work.
 *
 * To define a factor, derive an instance of this template and define
 * `calculate()` with the appropriate parameters.
 *
 * @tparam NumResiduals The dimension of the residual vector
 * @tparam VariableTypes The list of variables typed operated on by this factor
 */
template <int NumResiduals, typename... VariableTypes>
class Factor : public FactorBase {
    using FactorType = Factor<NumResiduals, VariableTypes...>;

 public:
    constexpr static int NumVars = sizeof...(VariableTypes);
    constexpr static int ResidualSize = NumResiduals;
    using ResidualType = Eigen::Matrix<double, NumResiduals, 1>;

    explicit Factor(std::shared_ptr<VariableTypes>... variables)
        : variables{variables...} {}
    virtual ~Factor() = default;

    /**
     * Calculate residuals and jacobians at the given variable values.
     * Override this function with the correct parameters for your derived type.
     *
     * @param[in] variables each variable this factor links
     * @param[out] residuals residuals calculated using the given variables
     * @param[out] jacobians
     * @parblock
     * Jacobian matrices with respect to each variable. There is one jacobian
     * parameter for each variable parameter, in the same order.
     * @endparblock
     *
     * @note  Each residual and jacobian parameter is a special map object which
     * may evaluate to false to indicate the jacobian should not be calculated
     * on this call. Wrap calculation and assignment of each parameter `j` in a
     * conditional, `if (j) {...}`.
     *
     * @return true if calculation was successful, false on failure
     */
    virtual bool evaluate(
      const VariableTypes &... variables,
      ResidualsOut<NumResiduals> residuals,
      JacobianOut<NumResiduals,
                  VariableTypes::SizeAtCompileTime>... jacobians) noexcept = 0;

    bool evaluateRaw(double const *const *parameters,
                     double *residuals,
                     double **jacobians) noexcept override;

    void print(std::ostream &os) const override;

 private:
    boost::fusion::vector<std::shared_ptr<VariableTypes>...> variables;
};

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif
