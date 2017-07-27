// Implementation of types declared in wave/utils/template_helpers.hpp

namespace wave {
namespace tmp {

// Generate an index_sequence<S, ..., S + N - 1> via recursion
// Count down in the first argument while adding one number to the start of the
// sequence.
template <int N, int S, int... Indices>
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


namespace internal {
template <int I = 0>
struct array_sum_impl {
    template <typename T, std::size_t N>
    constexpr T operator()(std::array<T, N> array) const {
        return std::get<I - 1>(array) + array_sum_impl<I - 1>{}(array);
    }
};

template <>
struct array_sum_impl<0> {
    template <typename T, std::size_t N>
    constexpr T operator()(std::array<T, N>) const {
        return 0;
    }
};

template <int I>
struct cumulative_array_impl {
    template <typename T, std::size_t N>
    constexpr T operator()(std::array<T, N> array) const {
        return std::get<I - 1>(array) + array_sum_impl<I - 1>{}(array);
    }
};

template <typename T, std::size_t N, int... Is>
constexpr std::array<T, N> cumulativeArrayImpl(std::array<T, N> in,
                                               index_sequence<Is...>) {
    // For each element, get the sum up to that element
    return std::array<T, N>{{array_sum_impl<Is>{}(in)...}};
}
}  // namespace internal


template <typename T, std::size_t N>
constexpr T array_sum(std::array<T, N> array) {
    return internal::array_sum_impl<N>{}(array);
}

template <typename T, std::size_t N>
constexpr std::array<T, N> cumulative_array(std::array<T, N> in) {
    return internal::cumulativeArrayImpl(in, make_index_sequence<N>{});
};

}  // namespace tmp
}  // namespace wave
