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

    /** Default construct with uninitialized estimate */
    FactorVariable() = default;

    /** Construct with initial value */
    explicit FactorVariable(MappedType &&initial)
        : storage{std::move(initial)} {}

    /** Construct copying initial value */
    explicit FactorVariable(const MappedType &initial) : storage{initial} {}

    /** Return the number of scalar values in the variable. */
    int size() const noexcept override {
        return Size;
    }

    /** Return a raw pointer to the start of the internal storage. */
    const double *data() const noexcept override {
        return this->storage.data();
    }
    double *data() noexcept override {
        return this->storage.data();
    }

    /** Marks as constant during optimization
     * @todo replace with unary factors for priors
     * */
    void setFixed(bool c) noexcept {
        this->fixed = c;
    }

    /** Whether this has been marked constant during optimization
     * @todo replace with unary factors for priors
     */
    bool isFixed() const noexcept override {
        return this->fixed;
    }

    void print(std::ostream &os) const override {
        os << "FactorVariable";
    }

 private:
    /** Internal buffer holding the parameter estimate */
    MappedType storage;

 public:
    /** */
    ViewType value{storage.data()};

 private:
    bool fixed = false;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
