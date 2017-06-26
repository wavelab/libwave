/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/template_helpers.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

template <typename T>
struct Map {};

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
template <typename Scalar, int S>
struct FactorValue : Eigen::Matrix<Scalar, S, 1> {
    constexpr static int Size = S;
    using Eigen::Matrix<Scalar, S, 1>::Matrix;
};

template <typename Scalar, int S>
struct FactorValue<Map<Scalar>, S> : Eigen::Map<FactorValue<Scalar, S>> {
    using Eigen::Map<FactorValue<Scalar, S>>::Map;
    using Eigen::Map<FactorValue<Scalar, S>>::operator=;
};

template <typename T, template <typename> class... ValueTypes>
class ComposedValue {
 public:
    using ValueTuple = std::tuple<ValueTypes<T>...>;
    using ValueSizes = tmp::index_sequence<ValueTypes<T>::Size...>;

    ComposedValue() : elements{ValueTypes<T>::Zero()...} {}
    explicit ComposedValue(ValueTypes<T>... args)
        : elements{std::move(args)...} {}

 protected:
    ValueTuple elements;
};

template <typename Scalar, template <typename> class... ValueTypes>
class ComposedValue<Map<Scalar>, ValueTypes...> {
 public:
    using ValueTuple = std::tuple<ValueTypes<Map<Scalar>>...>;
    using ValueSizes = tmp::index_sequence<ValueTypes<Scalar>::Size...>;
    explicit ComposedValue(tmp::replacet<Scalar *, ValueTypes>... args)
        : elements{ValueTypes<Map<Scalar>>{args}...} {}

 protected:
    ValueTuple elements;
};

/** Generic instances */
template <typename T>
using FactorValue1 = FactorValue<T, 1>;
template <typename T>
using FactorValue2 = FactorValue<T, 2>;
template <typename T>
using FactorValue3 = FactorValue<T, 3>;

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
