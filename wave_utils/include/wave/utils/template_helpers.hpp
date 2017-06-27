/**
 * @file
 * @ingroup optimization
 * Utility functions for template metaprogramming
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP


namespace wave {
/** @addtogroup optimization
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

}  // namespace tmp
/** @} group optimization */
}  // namespace wave

#include "impl/template_helpers.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
