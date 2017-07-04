/**
 * @file
 * @ingroup utils
 * Utility functions for template metaprogramming
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP


namespace wave {
/** @addtogroup utils
 *  @{ */

namespace tmp {

/**
 * Directly replace a type.
 *
 * Meant to be used to make parameter packs of a single type, of the same
 * number of arguments as another type parameter pack.
 *
 * For example, `replace<int, T>...` becomes `int...` of the same length.
 */
template <typename To, typename From>
using replace = To;

/**
 * Directly replace a type template.
 *
 * Meant to be used to make parameter packs of a single type, of the same
 * number of arguments as a template parameter pack.
 *
 * For example, `replace<int, T>...` becomes `int...` of the same length.
 */
template <typename To, template <typename...> class From>
using replacet = To;

/** We have an integer `N`, known at compiler time, and want the compiler to
 * generate a function call `f(1, ..., N)`. For example, if N is 3, we want to
 * call `f(1, 2, 3)`.
 *
 * The solution involves variadic templates. This template, `index_sequence`,
 * holds the sequence 1..N as template parameters. When we pass an instance of
 * it to a variadic function template, the sequence can become a parameter pack.
 * For example,
 *
 * ```
 * template<int... Is>
 * void callF(index_sequence<Is...> indices) { f(Is...); }
 *
 * int main() {
 *     callF(index_sequence<1, 2, 3>{});
 *     f(1, 2, 3);
 * }
 * ```
 *
 * The call above is equivalent to f(1, 2, 3). Of course, it's not useful if you
 * have to type out index_sequence<1, 2, 3>. That's why we define the template
 * `make_index_sequence`, which lets us call `callF(make_index_sequence<3>{});`.
 *
 * This approach is common enough that `index_sequence` and
 * `make_index_sequence` were added to the standard library in C++14. Since
 * they are not in C++11, we define a simple implementation here.
 *
 * See also:
 *
 * - http://en.cppreference.com/w/cpp/utility/integer_sequence
 * - The "indices trick": http://loungecpp.wikidot.com/tips-and-tricks:indices
 * - Variadic templates:
 * http://eli.thegreenplace.net/2014/variadic-templates-in-c
 */
template <int... Indices>
struct index_sequence {
    using type = index_sequence<Indices...>;
};

/** Concatenates any number of index sequences into one */
template <typename... Seqs>
struct concat_index_sequence;

/** Sums up the arguments.
 *  The result is placed in the static member `value`.
 */
template <int...>
struct sum;

/** Sums up the integers in an index sequence */
template <typename Seq>
struct sum_index_sequence;

/** Holds the cumulative index of each input in the input sequence. For example,
 * for an input index_sequence<2, 3, 3>, the output would be
 * index_sequence<0, 2, 5>.
 */
template <typename Seq, typename Out = index_sequence<0>, int Count = 0>
struct cumulative_index;

/** Gets one element of an index sequence */
template <int I, typename Seq>
struct index_sequence_element;

/** Constructs a vector from an index sequence
 * Note this is a runtime function.
 */
template <int... Is>
inline std::vector<int> vectorFromSequence(index_sequence<Is...>) {
    return {Is...};
}

/** A container for a sequence of types */
template <typename... Types>
struct type_sequence {
    using type = type_sequence<Types...>;
};

/** Inspects function argument and return types */
template <typename T>
struct function_traits;

/** Get the result type of concatenating some tuples */
template <typename... Tuples>
using tuple_cat_result =
  typename function_traits<decltype(std::tuple_cat<Tuples...>) *>::return_type;

/** Apply the functor F to each element in a tuple in-place
 * F::operator() should return void.
 */
template <typename Tuple, typename F>
void applyToTuple(Tuple &tuple, const F &f);

/* The return type of `transformTuple` with arguments of the given type */
template <typename, typename, typename = void>
struct result_of_transform;

template <typename... T>
using result_of_transform_t = typename result_of_transform<T...>::type;

/** Apply the functor F to each element in a tuple and return the new tuple */
template <typename Tuple, typename F>
result_of_transform_t<Tuple, F> transformTuple(const Tuple &tuple, const F &f);

/** Apply the functor F to each pair of elements in two tuples
 * @return the new tuple */
template <typename A, typename B, typename F>
result_of_transform_t<A, B, F> transformTuple(const A &tuple_a,
                                              const B &tuple_b,
                                              const F &f);

/* The return type of `transformTuple` with arguments of the given type */
template <typename, template <typename> class>
struct result_of_transform_tmpl;

template <typename T, template <typename> class F>
using result_of_transform_tmpl_t =
  typename result_of_transform_tmpl<T, F>::type;

/** Apply the templated functor F to each pair of elements in two tuples
 *
 * This is meant to be used with function objects such as std::plus and
 * std::minus, which are class templates (as opposed to classes with a
 * templated operator() )
 *
 * @return the new tuple
 */
template <template <typename> class F, typename T>
result_of_transform_tmpl_t<T, F> transformTupleTmpl(const T &tuple_a,
                                                    const T &tuple_b);

/** Gets the result of instantiating the template F with each type in Seq

 * F must define a boolean member `value`. Then,
 * `check_all<type_sequence<A, B, C>, F>::value` is equivalent to
 * `F<A>::value && F<B>::value && F<C>::value`.
 */
template <typename Seq, template <typename> class F>
struct check_all;

/** Remove pointer-to-member and const qualification (if const) from a
 * pointer-to-member-function type.
 *
 * For example, `int (Foo::*)(int, int)` becomes `(int)(int, int)`
 * */
template <typename T>
struct clean_method;

/** Get sum of an std::array */
template <typename T, std::size_t N>
constexpr T array_sum(std::array<T, N> array);

/** Return an array with the cumulative index of each input in the input array.
 * For example, for an input {2, 3, 3}, the output would be {0, 2, 5}.
 */
template <typename T, std::size_t N>
constexpr std::array<T, N> cumulative_array(std::array<T, N> in);

}  // namespace tmp
/** @} group utils */
}  // namespace wave

#include "impl/template_helpers.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
