/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <ceres/sized_cost_function.h>
#include <memory>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"
#include "wave/optimization/factor_graph/OutputMap.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

template <int NumResiduals, int... VariableSizes>
class FactorCostFunction
  : public ceres::SizedCostFunction<NumResiduals, VariableSizes...> {
 public:
    explicit FactorCostFunction(
      std::function<bool(double const *const *, double *, double **)> fn)
        : f_evaluate{std::move(fn)} {}

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override {
        return f_evaluate(parameters, residuals, jacobians);
    }

 private:
    std::function<bool(double const *const *, double *, double **)> f_evaluate;
};

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
 * @tparam VarTypes The list of variables typed operated on by this factor
 */
template <int NumResiduals, typename... VarTypes>
class Factor : public FactorBase {
    using FactorType = Factor<NumResiduals, VarTypes...>;

 public:
    constexpr static int NumVars = sizeof...(VarTypes);
    constexpr static int ResidualSize = NumResiduals;
    using ResidualType = Eigen::Matrix<double, NumResiduals, 1>;
    using VarArrayType = FactorBase::VarVectorType;
    using const_iterator = typename VarArrayType::const_iterator;

    /** Construct the factor with the given variables */
    explicit Factor(std::shared_ptr<VarTypes>... variable_ptrs)
        : variable_ptrs{{variable_ptrs...}} {}

    ~Factor() override = default;

    int size() const override {
        return NumVars;
    }
    int numResiduals() const override {
        return NumResiduals;
    }

    std::unique_ptr<ceres::CostFunction> costFunction() override {
        auto fn = std::bind(&FactorType::evaluateRaw,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3);
        return std::unique_ptr<ceres::CostFunction>{
          new FactorCostFunction<NumResiduals, VarTypes::ViewType::Size...>{
            fn}};
    }

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
     * conditional, `if (j) {...}`.R
     *
     * @return true if calculation was successful, false on failure
     */
    virtual bool evaluate(
      const typename VarTypes::ViewType &... variables,
      ResidualsOut<NumResiduals> residuals,
      JacobianOut<NumResiduals, VarTypes::Size>... jacobians) const
      noexcept = 0;

    bool evaluateRaw(double const *const *parameters,
                     double *residuals,
                     double **jacobians) const noexcept override;


    /** Get a reference to the vector of variable pointers */
    const VarVectorType &variables() const noexcept override {
        return this->variable_ptrs;
    }

    /** Print a representation for debugging. Used by operator<< */
    void print(std::ostream &os) const override;

 private:
    /** Pointers to the variables this factor is linked to */
    VarArrayType variable_ptrs;
};

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif
