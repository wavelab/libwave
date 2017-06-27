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

/** Generates an index_sequence<S, ..., S + N - 1> via recursion */
template <int N, int S = 0, int... Indices>
struct make_index_sequence
  : make_index_sequence<N - 1, S, S + N - 1, Indices...> {};

// Final stop on the recursion train
template <int S, int... Indices>
struct make_index_sequence<0, S, Indices...> : index_sequence<Indices...> {};

/** Generates an index_sequence with N repetitions of S */
template <int N, int S, int... Indices>
struct repeat_index_sequence : repeat_index_sequence<N - 1, S, S, Indices...> {
};

// Final stop on the recursion train
template <int S, int... Indices>
struct repeat_index_sequence<0, S, Indices...> : index_sequence<Indices...> {};

/** Concatenates index sequences */
template <typename... Seqs>
struct concat_index_sequence;

template <int... I1, int... I2, typename... Seqs>
struct concat_index_sequence<index_sequence<I1...>,
                             index_sequence<I2...>,
                             Seqs...>
  : concat_index_sequence<index_sequence<I1..., I2...>, Seqs...> {};

template <int... I1>
struct concat_index_sequence<index_sequence<I1...>> {
    using type = index_sequence<I1...>;
};

/** Sum up the arguments */
template <int...>
struct sum;

template <int Head, int... Tail>
struct sum<Head, Tail...> {
    static constexpr int value = Head + sum<Tail...>::value;
};

template <int Head>
struct sum<Head> {
    static constexpr int value = Head;
};


/** Sums up an index sequence> */
template <typename Seq>
struct sum_index_sequence;

template <int Head, int... Is>
struct sum_index_sequence<index_sequence<Is...>> {
    static constexpr int value = sum<Is...>::value;
};

/** Construct a vector from an index sequence */
template <int... Is>
inline std::vector<int> vectorFromSequence(index_sequence<Is...>) {
    return {Is...};
}

/** A container for a sequence of types */
template <typename... Types>
struct type_sequence {
    using type = type_sequence<Types...>;
};

/** Extracts the element at position I from the type_sequence */
template <int I, typename Seq>
struct type_sequence_element;

// Start of recursion. Move head of sequence forward as S counts down
template <int I, typename Head, typename... Tail>
struct type_sequence_element<I, type_sequence<Head, Tail...>>
  : type_sequence_element<I - 1, type_sequence<Tail...>> {};

//  Base case
template <typename Head, typename... Tail>
struct type_sequence_element<0, type_sequence<Head, Tail...>> {
    using type = Head;
};

/** A container for a sequence of templates */
template <template <typename...> class... Templates>
struct tmpl_sequence {};

/** Concatenates template sequences */
template <typename... Seqs>
struct concat_tmpl_sequence;

// Recurse forwards, always adding the second sequence to the first
template <template <typename...> class... T1,
          template <typename...> class... T2,
          typename... Seqs>
struct concat_tmpl_sequence<tmpl_sequence<T1...>, tmpl_sequence<T2...>, Seqs...>
  : concat_tmpl_sequence<tmpl_sequence<T1..., T2...>, Seqs...> {};

// Base case: only one sequence left
template <template <typename...> class... T1>
struct concat_tmpl_sequence<tmpl_sequence<T1...>> {
    using type = tmpl_sequence<T1...>;
};


/** Inspects function argument and return types */
template <typename T>
struct function_traits;

template <typename R, typename... Args>
struct function_traits<R (*)(Args...)> {
    constexpr static int arity = sizeof...(Args);
    using return_type = R;

    template <int I>
    using arg_type = typename std::tuple_element<I, std::tuple<Args...>>::type;

    using arg_types = std::tuple<Args...>;
};


/** Get the result type of concatenating some tuples */
template <typename... Tuples>
using tuple_cat_result =
  typename function_traits<decltype(std::tuple_cat<Tuples...>) *>::return_type;

}  // namespace tmp
/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
