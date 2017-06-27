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

template <int ResidualSize, int... VariableSizes>
class FactorCostFunction
  : public ceres::SizedCostFunction<ResidualSize, VariableSizes...> {
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
template <typename MeasType, typename... VarTypes>
class Factor : public FactorBase {
    using FactorType = Factor<MeasType, VarTypes...>;

 public:
    constexpr static int NumVars = sizeof...(VarTypes);
    constexpr static int ResidualSize = MeasType::Size;
    using ResidualType = Eigen::Matrix<double, ResidualSize, 1>;
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
    using FuncType = bool(const typename VarTypes::ViewType &...,
                          ResultOut<ResidualSize>,
                          JacobianOut<ResidualSize, VarTypes::Size>...);

    using const_iterator = typename VarArrayType::const_iterator;

    /** Construct with the given function, measurement and variables. */
    explicit Factor(FuncType f,
                    MeasType meas,
                    std::shared_ptr<VarTypes>... variable_ptrs)
        : measurement_function{f},
          measurement{meas},
          variable_ptrs{{variable_ptrs...}} {}

    ~Factor() override = default;

    int size() const override {
        return NumVars;
    }
    int numResiduals() const override {
        return ResidualSize;
    }

    std::unique_ptr<ceres::CostFunction> costFunction() override {
        auto fn = std::bind(&FactorType::evaluateRaw,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3);
        return std::unique_ptr<ceres::CostFunction>{
          new FactorCostFunction<ResidualSize, VarTypes::ViewType::Size...>{
            fn}};
    }

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
    FuncType *measurement_function;

    /** Storage of the measurement */
    MeasType measurement;

    /** Pointers to the variables this factor is linked to */
    VarArrayType variable_ptrs;
};

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif
