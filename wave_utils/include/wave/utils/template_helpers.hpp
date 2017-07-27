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

/** Constructs an index_sequence<S, S+1, ..., S+N-1> */
template <int N, int S = 0, int...>
struct make_index_sequence;

/** Concatenates any number of index sequences into one */
template <typename... Seqs>
struct concat_index_sequence;

/** Sums up the arguments.
 *  The result is placed in the static member `value`.
 *
 *  For example, `sum<1, 2, 3>::value` evaluates to `6`.
 */
template <int...>
struct sum;

/** Sums up the integers in an index sequence
 * The result is placed in the static member `value`.
 *
 * For example, `sum_index_sequence<index_sequence<1, 2, 3>>::value` evaluates
 * to `6`.
 * */
template <typename Seq>
struct sum_index_sequence;

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
