/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/ValueView.hpp"

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
template <typename V>
class FactorVariable : public FactorVariableBase {
 public:
    using ViewType = V;
    using MappedType = typename ViewType::MappedType;
    constexpr static int Size = ViewType::Size;

    // Constructors

    /** Default construct with uninitialized estimate
     * Actually initializes to zero to avoid problems with garbage floats
     * @todo move setting of initial value elsewhere
     */
    FactorVariable() noexcept : storage{MappedType::Zero()}, value{storage} {}

    /** Construct with initial value */
    explicit FactorVariable(MappedType initial)
        : storage{std::move(initial)}, value{storage} {}

    /** Construct with initial value, only for variables of size one*/
    explicit FactorVariable(double initial)
        : storage{initial}, value{storage} {}

    // Because `value` is a map holding a pointer to another member, we must
    // define a custom copy constructor (and the rest of the rule of five)
    /** Copy constructor */
    FactorVariable(const FactorVariable &other) noexcept
      : FactorVariable{other.storage} {}

    /** Move constructor */
    FactorVariable(FactorVariable &&other) noexcept
      : FactorVariable{std::move(other.storage)} {}

    /** Copy assignment operator */
    FactorVariable &operator=(const FactorVariable &other) noexcept {
        auto temp = FactorVariable{other};
        swap(*this, temp);
        return *this;
    }

    /** Move assignment operator */
    FactorVariable &operator=(FactorVariable &&other) noexcept {
        auto temp = std::move(other);
        swap(*this, temp);
        return *this;
    }

    friend void swap(FactorVariable &lhs, FactorVariable &rhs) noexcept {
        std::swap(lhs.storage, rhs.storage);
        // Note: don't swap value
    }

    ~FactorVariable() override = default;

    // Access

    /** Return the number of scalar values in the variable. */
    int size() const noexcept override {
        return Size;
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

 private:
    /** Internal buffer holding the parameter estimate */
    MappedType storage;

 public:
    /** */
    ViewType value;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
