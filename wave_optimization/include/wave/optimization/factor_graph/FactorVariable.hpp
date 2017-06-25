/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/FactorValue.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

using VariableId = int;

/**
 * Representation of a variable in a factor graph.
 *
 *
 * A FactorVariable has a value, which is accessed through a ValueView. Note the
 * difference between variable, value, and view:
 *
 * In a factor graph, a _variable_ is a node representing an random variable
 * whose value must be estimated. A variable typically has some physical meaning
 * - for example, a pose in 3D space - and has a sample space - for example, the
 * space of 6D vectors.
 *
 * A _value_ is a specific point in that space, for example, a 6D vector
 * representing a particular position and orientation. A value might be stored
 * in an Eigen `Vector` object or a raw array of doubles.
 *
 * While variables and values are fundamental to the problem, a _view_ is a
 * syntactic convenience. It is a data structure which assigns labels to an
 * underlying value, to simplify interacting with it. For example, a Pose3D view
 * may define elements of the underlying 6D value as `position` and
 * `orientation`. In the Factor's evaluation function, these objects can then be
 * used as independent Eigen vectors.
 */
template <template <typename> class V>
class FactorVariable : public FactorVariableBase {
 public:
    using ValueType = V<double>;

    // Constructors

    /** Default construct with uninitialized estimate
     * Actually initializes to zero to avoid problems with garbage floats
     * @todo move setting of initial value elsewhere
     */
    FactorVariable() noexcept : value{ValueType::Zero()} {}

    /** Construct with initial value */
    explicit FactorVariable(ValueType initial)
        : value{std::move(initial)} {}

    /** Construct with initial value, only for variables of size one*/
    explicit FactorVariable(double initial)
        : value{initial} {}

    ~FactorVariable() override = default;

    // Access

    /** Return the number of scalar values in the variable. */
    int size() const noexcept override {
        return 0;
    }

    /** Return a raw pointer to the start of the internal storage. */
    const double *data() const noexcept override {
        return this->value.data();
    }
    double *data() noexcept override {
        return this->value.data();
    }

    void print(std::ostream &os) const override {
        os << "FactorVariable";
    }

    /** The actual value buffer */
    ValueType value;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
