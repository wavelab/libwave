/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP

#include <iostream>
#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/utils/template_helpers.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

namespace internal {

/** Traits template for FactorValue types.
 *
 * It is used in templates that can accept FactorValue%s and ComposedValue%s
 * interchangably.
 *
 * @tparam V an instantiated FactorValue or ComposedValue class
 *
 * The other parameter is used internally for deduction.
 */
template <typename V, typename = void>
struct factor_value_traits;

}  // namespace internal

struct FactorValueOptions {
    struct Map;
    struct Square;
};

/**
 * A structure mapping useful types to an underlying value buffer.
 *
 * While variables and values are fundamental to the factor graph approach, a
 * _view_ is a syntactic convenience. It is a data structure which assigns
 * labels to an underlying value, to simplify interacting with it. For example,
 * a Pose3D view may define elements of the underlying 6D value as `position`
 * and `orientation`. In the Factor's evaluation function, these objects can
 * then be used as independent Eigen vectors.
 *
 * This class is meant to be derived by specialized variable types which assign
 * meaning to the underlying data vector.
 *
 * @todo add more syntactic sugar
 */
template <typename Scalar, typename Options, int S>
class FactorValue : public Eigen::Matrix<Scalar, S, 1> {
 public:
    constexpr static int Size = S;
    using Base = Eigen::Matrix<Scalar, Size, 1>;

    // Inherit base constructors and assignment
    using Base::Base;
    using Base::operator=;

    // Must redefine default constructor since we define other constructors
    FactorValue() : Base{} {}

    /** Allow constructing a size-1 value from a scalar */
    FactorValue(Scalar d) : Base{d} {}

    /** Allow assigning to a size-1 value from a scalar */
    FactorValue &operator=(Scalar d) {
        static_assert(Size == 1, "Can only assign scalar to values of size 1");
        (*this)[0] = d;
        return *this;
    }
};

template <typename Scalar, int S>
class FactorValue<Scalar, FactorValueOptions::Map, S>
  : public Eigen::Map<Eigen::Matrix<Scalar, S, 1>> {
 public:
    constexpr static int Size = S;
    using Base = Eigen::Map<Eigen::Matrix<Scalar, Size, 1>>;

    // Inherit base constructors and assignment
    using Base::Base;
    using Base::operator=;

    /** Allow assigning to a size-1 value from double */
    FactorValue &operator=(Scalar d) {
        static_assert(Size == 1, "Can only assign scalar to values of size 1");
        (*this)[0] = d;
        return *this;
    }
};

/** Specialization for Square option */
template <typename Scalar, int S>
class FactorValue<Scalar, FactorValueOptions::Square, S>
  : public Eigen::Matrix<Scalar, S, S> {
 public:
    constexpr static int Size = S;
    using Base = Eigen::Matrix<Scalar, Size, Size>;

    // Inherit base constructors and assignment
    using Base::Base;
    using Base::operator=;

    // Must redefine default constructor since we define other constructors
    FactorValue() : Base{} {}

    /** Allow constructing a size-1 value from a scalar */
    FactorValue(Scalar d) : Base{d} {}

    /** Allow assigning to a size-1 value from a scalar */
    FactorValue &operator=(Scalar d) {
        static_assert(Size == 1, "Can only assign scalar to values of size 1");
        (*this)[0] = d;
        return *this;
    }
};

/** Generic instances */
template <typename T, typename O = void>
using FactorValue1 = FactorValue<T, O, 1>;
template <typename T, typename O = void>
using FactorValue2 = FactorValue<T, O, 2>;
template <typename T, typename O = void>
using FactorValue3 = FactorValue<T, O, 3>;


/** @} group optimization */
}  // namespace wave

#include "impl/FactorValue.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
