// Implementation of types declared in wave/utils/template_helpers.hpp

namespace wave {
namespace tmp {

// Generate an index_sequence<S, ..., S + N - 1> via recursion
// Count down in the first argument while adding one number to the start of the
// sequence.
template <int N, int S = 0, int... Indices>
struct make_index_sequence
  : make_index_sequence<N - 1, S, S + N - 1, Indices...> {};

// Base case: the counter reached S.
// Remove the counter, leaving only the finished sequence.
template <int S, int... Indices>
struct make_index_sequence<0, S, Indices...> : index_sequence<Indices...> {};

// Recursively combine any number of index sequences
// Concatenate two at a time. Always append the indices from the second sequence
// onto the first, and discard the second sequence.
template <int... I1, int... I2, typename... Seqs>
struct concat_index_sequence<index_sequence<I1...>,
                             index_sequence<I2...>,
                             Seqs...>
  : concat_index_sequence<index_sequence<I1..., I2...>, Seqs...> {};

// Base case: one sequence left
template <int... I1>
struct concat_index_sequence<index_sequence<I1...>> {
    using type = index_sequence<I1...>;
};

// Recursively find the sum of any number of integers
// Add the first int to the sum of the remaining ints
template <int Head, int... Tail>
struct sum<Head, Tail...> {
    static constexpr int value = Head + sum<Tail...>::value;
};

// Base case: reached the last int in the sequence
template <int Tail>
struct sum<Tail> {
    static constexpr int value = Tail;
};

// Apply `sum` to the integers in an index_sequence
template <int... Is>
struct sum_index_sequence<index_sequence<Is...>> {
    static constexpr int value = sum<Is...>::value;
};

// Specialization of function_traitss for regular function pointer
template <typename R, typename... Args>
struct function_traits<R (*)(Args...)> {
    constexpr static int arity = sizeof...(Args);
    using return_type = R;

    template <int I>
    using arg_type = typename std::tuple_element<I, std::tuple<Args...>>::type;

    using arg_types = std::tuple<Args...>;
};

// Recurse forward along the sequence
template <typename Head, typename... Tail, template <typename> class F>
struct check_all<type_sequence<Head, Tail...>, F> {
    static constexpr bool value =
      F<Head>::value && check_all<type_sequence<Tail...>, F>::value;
};

// Base case: last element of sequence
template <typename Head, template <typename> class F>
struct check_all<type_sequence<Head>, F> {
    static constexpr bool value = F<Head>::value;
};

template <typename T>
struct remove_member_pointer {
    using type = T;
};
template <typename T, typename U>
struct remove_member_pointer<T U::*> {
    using type = T;
};

template <typename T>
struct remove_method_const {
    using type = T;
};

template <typename R, typename... Args>
struct remove_method_const<R(Args...) const> {
    using type = R(Args...);
};

template <typename T>
struct clean_method {
    using type = typename remove_method_const<
      typename remove_member_pointer<T>::type>::type;
};

}  // namespace tmp
}  // namespace wave
