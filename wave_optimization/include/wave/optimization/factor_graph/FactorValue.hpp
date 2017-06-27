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

namespace internal {

template <typename T, typename O, int S>
struct FactorValueAlias;

template <typename V, typename = void>
struct factor_value_traits {
    constexpr static int NumVals = 1;
    using BlockSizes = tmp::index_sequence<V::SizeAtCompileTime>;
    constexpr static int TotalSize = V::SizeAtCompileTime;
    static std::vector<double *> blockData(V &v) {
        return {v.data()};
    }
};

template <typename, typename>
struct factor_value_traits_nested;

template <typename V>
struct factor_value_traits<
  V,
  std::conditional<false, void, typename V::ValueTuple>>
  : factor_value_traits_nested<V, typename V::ValueTuple> {};

template <typename V, typename... Nested>
struct factor_value_traits_nested<V, std::tuple<Nested...>> {
    constexpr static int NumVals = sizeof...(Nested);
    using BlockSizes = typename tmp::concat_index_sequence<
      typename factor_value_traits<Nested>::BlockSizes...>::type;
    constexpr static int TotalSize = tmp::sum_index_sequence<BlockSizes>::value;
    static std::vector<double *> blockData(V &v) {
        return {v.blockData()};
    }
};


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
using FactorValue =
  typename internal::FactorValueAlias<Scalar, Options, Size>::type;

template <typename Scalar, typename Options, template <typename...> class... V>
class ComposedValue {
 public:
    using ValueSizes = typename tmp::concat_index_sequence<
      typename V<Scalar, Options>::ValueSizes...>::type;
    using ValueTuple = std::tuple<V<Scalar, Options>...>;
    using ValueTmpls = tmp::tmpl_sequence<V...>;
    constexpr static int NumValues = sizeof...(V);

    ComposedValue() : blocks{V<Scalar, Options>::Zero()...} {}
    explicit ComposedValue(V<Scalar, Options>... args)
        : blocks{std::move(args)...} {}

    explicit ComposedValue(tmp::replacet<Scalar *, V>... args)
        : blocks{V<Scalar, FactorValueOptions::Map>{args}...} {}

    template <int... Is>
    std::vector<Scalar *> blockDataImpl(tmp::index_sequence<Is...>) noexcept {
        return {std::get<Is>(this->blocks).data()...};
    }

    std::vector<Scalar *> blockData() noexcept {
        return this->blockDataImpl(tmp::make_index_sequence<NumValues>{});
    }

 protected:
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

template <template <typename...> class... ComposedOrValues>
using get_value_sizes = typename tmp::concat_index_sequence<
  typename ComposedOrValues<double>::ValueSizes...>::type;

// template <template <typename...> class... ComposedOrValues>
// using get_value_tmpls = typename tmp::tmpl_sequence<
//  typename ComposedOrValues<double>::ValueTmpls...>::type;

template <template <typename...> class... ComposedOrValues>
using expand_value_tmpls = typename tmp::concat_tmpl_sequence<
  typename ComposedOrValues<double>::ValueTmpls...>::type;

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

template <template <typename...> class... ComposedOrValues>
using get_value_indices = typename get_value_indices_impl<
  0,
  tmp::index_sequence<ComposedOrValues<double>::NumValues...>>::type;

}  // namespace internal


/** @} group optimization */
}  // namespace wave

#include "impl/FactorValue.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
