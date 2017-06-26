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
    template <typename U>
    using Tmpl = FactorValue<U, S>;
    using ValueSizes = tmp::index_sequence<S>;
    using ValueTuple = std::tuple<FactorValue<Scalar, S>>;
    using ValueTmpls = tmp::tmpl_sequence<Tmpl>;
    constexpr static int NumValues = 1;
    constexpr static int Size = S;
    using Eigen::Matrix<Scalar, S, 1>::Matrix;

    std::vector<int> blockSizes() const noexcept {
        return {Size};
    }
    std::vector<Scalar *> blockData() noexcept {
        return {this->data()};
    }
};

template <typename Scalar, int S>
struct FactorValue<Map<Scalar>, S> : Eigen::Map<FactorValue<Scalar, S>> {
    template <typename U>
    using Tmpl = FactorValue<U, S>;
    using ValueSizes = tmp::index_sequence<S>;
    using ValueTuple = std::tuple<FactorValue<Map<Scalar>, S>>;
    using ValueTmpls = tmp::tmpl_sequence<Tmpl>;
    constexpr static int NumValues = 1;
    constexpr static int Size = S;
    using Eigen::Map<FactorValue<Scalar, S>>::Map;
    using Eigen::Map<FactorValue<Scalar, S>>::operator=;

    std::vector<int> blockSizes() const noexcept {
        return {Size};
    }
    std::vector<Scalar *> blockData() noexcept {
        return {this->data()};
    }
};

template <typename T, template <typename> class... V>
class ComposedValue {
 public:
    using ValueSizes =
      typename tmp::concat_index_sequence<typename V<T>::ValueSizes...>::type;
    using ValueTuple = std::tuple<V<T>...>;
    using ValueTmpls = tmp::tmpl_sequence<V...>;
    constexpr static int NumValues = sizeof...(V);

    ComposedValue() : elements{V<T>::Zero()...} {}
    explicit ComposedValue(V<T>... args) : elements{std::move(args)...} {}

    std::vector<int> blockSizes() const noexcept {
        return {V<T>::Size...};
    }

    template <int... Is>
    std::vector<T *> blockDataImpl(tmp::index_sequence<Is...>) noexcept {
        return {std::get<Is>(this->elements).data()...};
    }

    std::vector<T *> blockData() noexcept {
        return this->blockDataImpl(tmp::make_index_sequence<NumValues>{});
    }

 protected:
    ValueTuple elements;
    std::vector<T *> dataptrs;
};

template <typename Scalar, template <typename> class... V>
class ComposedValue<Map<Scalar>, V...> {
 public:
    using ValueSizes = typename tmp::concat_index_sequence<
      typename V<Map<Scalar>>::ValueSizes...>::type;
    using ValueTuple = std::tuple<V<Map<Scalar>>...>;
    using ValueTmpls = tmp::tmpl_sequence<V...>;
    constexpr static int NumValues = sizeof...(V);

    explicit ComposedValue(tmp::replacet<Scalar *, V>... args)
        : elements{V<Map<Scalar>>{args}...} {}

    std::vector<int> blockSizes() const noexcept {
        return {V<Scalar>::Size...};
    }

    template <int... Is>
    std::vector<Scalar *> blockDataImpl(tmp::index_sequence<Is...>) noexcept {
        return {std::get<Is>(this->elements).data()...};
    }

    std::vector<Scalar *> blockData() noexcept {
        return this->blockDataImpl(tmp::make_index_sequence<NumValues>{});
    }

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


/** @todo figure this out */
namespace Eigen {
namespace internal {

template <typename Scalar, int S>
struct traits<wave::FactorValue<Scalar, S>>
  : traits<Eigen::Matrix<Scalar, S, 1>> {};

template <typename Scalar, int S>
struct traits<wave::FactorValue<wave::Map<Scalar>, S>>
  : traits<Eigen::Map<wave::FactorValue<Scalar, S>>> {};
}
}

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VALUE_HPP
