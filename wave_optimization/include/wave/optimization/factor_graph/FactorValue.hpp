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
template <typename Scalar, typename Options, int Size>
class FactorValue : public Eigen::Matrix<Scalar, Size, 1> {
 public:
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

template <typename Scalar, int Size>
class FactorValue<Scalar, FactorValueOptions::Map, Size>
  : public Eigen::Map<Eigen::Matrix<Scalar, Size, 1>> {
 public:
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

template <typename Scalar, typename Options, template <typename...> class... V>
class ComposedValue {
 public:
    using ValueTuple = std::tuple<V<Scalar, Options>...>;

    ComposedValue() : blocks{V<Scalar, Options>::Zero()...} {}

    explicit ComposedValue(V<Scalar, Options>... args)
        : blocks{std::move(args)...} {}

    explicit ComposedValue(tmp::replacet<Scalar *, V>... args)
        : blocks{V<Scalar, FactorValueOptions::Map>{args}...} {}
    std::vector<Scalar *> blockData() noexcept {
        return this->blockDataImpl(tmp::make_index_sequence<sizeof...(V)>{});
    }

 protected:
    template <int I>
    typename std::tuple_element<I, ValueTuple>::type &block() noexcept {
        return std::get<I>(this->blocks);
    }

 private:
    template <int... Is>
    std::vector<Scalar *> blockDataImpl(tmp::index_sequence<Is...>) noexcept {
        return {std::get<Is>(this->blocks).data()...};
    }

    ValueTuple blocks;
};

/** Generic instances */
template <typename T, typename O = void>
using FactorValue1 = FactorValue<T, O, 1>;
template <typename T, typename O = void>
using FactorValue2 = FactorValue<T, O, 2>;
template <typename T, typename O = void>
using FactorValue3 = FactorValue<T, O, 3>;


namespace internal {

// template <template <typename...> class ComposedOrValue>
// struct tmpl_type;
//
// template <template <typename, template <typename> class...> class Tmpl>
// struct tmpl_type {
//};
//
// template <template <typename... V> class C, true>
// struct tmpl_type {
//    using type = typename tmp::concat_tmpl_sequence<V...>::type;
//};

template <typename... ComposedOrValues>
using get_value_types =
  tmp::tuple_cat_result<typename ComposedOrValues::ValueTuple...>;

template <template <typename...> class... V>
using get_value_sizes = typename tmp::concat_index_sequence<
  typename internal::factor_value_traits<V<double>>::BlockSizes...>::type;

template <template <typename...> class... V>
using expand_value_tmpls = typename tmp::tuple_cat_result<
  typename internal::factor_value_traits<V<double>>::ValueTuple...>;

template <int S, typename ISeq, typename OutSeq = tmp::type_sequence<>>
struct get_value_indices_impl;

template <int S, int Head, int... Tail, typename... Out>
struct get_value_indices_impl<S,
                              tmp::index_sequence<Head, Tail...>,
                              tmp::type_sequence<Out...>>
  : get_value_indices_impl<
      S + Head,
      tmp::index_sequence<Tail...>,
      tmp::type_sequence<Out...,
                         typename tmp::make_index_sequence<Head, S>::type>> {};

template <int S, int Head, typename... Out>
struct get_value_indices_impl<S,
                              tmp::index_sequence<Head>,
                              tmp::type_sequence<Out...>>
  : tmp::type_sequence<Out...,
                       typename tmp::make_index_sequence<Head, S>::type> {};

template <template <typename...> class... V>
using get_value_indices = typename get_value_indices_impl<
  0,
  typename tmp::index_sequence<
    internal::factor_value_traits<V<double>>::NumValues...>>::type;

}  // namespace internal


/** @} group optimization */
}  // namespace wave

#include "impl/FactorValue.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
