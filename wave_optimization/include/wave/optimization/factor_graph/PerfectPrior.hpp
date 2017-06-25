/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_PERFECT_PRIOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_PERFECT_PRIOR_HPP

#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"
#include "wave/optimization/factor_graph/FactorMeasurement.hpp"


namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * A unary factor representing a noiseless prior
 *
 * A prior in a factor graph can be described as
 *
 * @f[
 * Z = f(\theta) + \epsilon
 * @f]
 *
 * where @f$ f @$f is the measurement function,  @f$ \theta @$f is a random
 * variable and @f$ \epsilon @$f is random noise. @f$ \epsilon @f cannot be
 * zero in general, as there may not be a solution for @f$ \Theta @$f.
 *
 * In libwave, noiseless priors are allowed in the case of a direct
 * measurement (i.e., @f$ f(\theta) = \theta @$f). These are called perfect
 * priors. Each FactorVariable may be connected to at most one perfect prior.
 *
 * @tparam VarType The type of factor variable we have prior information on
 */
template <template <typename> class V>
class PerfectPrior : public FactorBase {
    using FactorType = PerfectPrior<V>;
    using VarType = FactorVariable<V>;
    using ValueType = typename VarType::ValueType;
    using MeasType = FactorMeasurement<V, void>;

 public:
    constexpr static int NumVars = 1;
    constexpr static int ResidualSize = MeasType::Size;
    using ResidualType = Eigen::Matrix<double, ResidualSize, 1>;
    using VarArrayType = FactorBase::VarVectorType;
    using const_iterator = typename VarArrayType::const_iterator;

    /** Construct with the given measurement and variable. */
    explicit PerfectPrior(MeasType measurement,
                          std::shared_ptr<VarType> variable_ptr)
        : measurement{measurement}, variable_ptrs{variable_ptr} {
        // Assign to the variable
        variable_ptr->value = measurement.value;
    }

    int size() const override {
        return NumVars;
    }

    int residualSize() const override {
        return ResidualSize;
    }

    std::unique_ptr<ceres::CostFunction> costFunction() const
      noexcept override {
        throw std::logic_error("PerfectPrior has no cost function");
    }

    template <typename T>
    bool evaluateRaw(VarType const *const, T *) const noexcept {
        return false;
    }

    /** Get a reference to the vector of variable pointers */
    const VarVectorType &variables() const noexcept override {
        return this->variable_ptrs;
    }

    /** Return true if this factor is a zero-noise prior */
    bool isPerfectPrior() const noexcept override {
        return true;
    }

    /** Print a representation for debugging. Used by operator<< */
    void print(std::ostream &os) const override {
        os << "[Perfect prior for variable ";
        const auto &v = this->variable_ptrs.front();
        os << *v << "(" << v << ")]";
    }

 private:
    /** Storage of the measurement */
    MeasType measurement;

    /** Pointers to the variables this factor is linked to */
    VarArrayType variable_ptrs;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_PERFECT_PRIOR_HPP
