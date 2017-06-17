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
 * @tparam ResidualSize The dimension of the residual vector
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
    using FuncType = bool(const typename VarTypes::ViewType &...,
                          ResultOut<ResidualSize>,
                          JacobianOut<ResidualSize, VarTypes::Size>...);

    using const_iterator = typename VarArrayType::const_iterator;

    /** Construct the factor with the given measurement and variables */
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
