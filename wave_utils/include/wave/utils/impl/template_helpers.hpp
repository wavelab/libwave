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

// Recursively accumulate index
template <int Head, int... Tail, int... Res, int Count>
struct cumulative_index<index_sequence<Head, Tail...>,
                        index_sequence<Res...>,
                        Count>
  : cumulative_index<index_sequence<Tail...>,
                     index_sequence<Res..., Head + Count>,
                     Head + Count> {};

// Recurse, counting down from requested element
template <int I, int Head, int... Tail>
struct index_sequence_element<I, index_sequence<Head, Tail...>>
  : index_sequence_element<I - 1, index_sequence<Tail...>> {};

// Base case
template <int Head, int... Tail>
struct index_sequence_element<0, index_sequence<Head, Tail...>> {
    constexpr static int value = Head;
};

// Base case: the size of last entry doesn't matter
template <int Tail, int... Res, int Count>
struct cumulative_index<index_sequence<Tail>, index_sequence<Res...>, Count>
  : index_sequence<Res...> {};

// Specialization of function_traitss for regular function pointer
template <typename R, typename... Args>
struct function_traits<R (*)(Args...)> {
    constexpr static int arity = sizeof...(Args);
    using return_type = R;

    template <int I>
    using arg_type = typename std::tuple_element<I, std::tuple<Args...>>::type;

    using arg_types = std::tuple<Args...>;
};

namespace internal {
template <typename Tuple, typename F, int... Is>
void applyToTupleHelper(Tuple &tuple, const F &f, index_sequence<Is...>) {
    auto loop = {(f(std::get<Is>(tuple)), 0)...};
    (void) loop;
}

template <typename Tuple, typename F, int... Is>
result_of_transform_t<Tuple, F> transformTupleHelper(const Tuple &tuple,
                                                     const F &f,
                                                     index_sequence<Is...>) {
    return result_of_transform_t<Tuple, F>{f(std::get<Is>(tuple))...};
}

template <typename A, typename B, typename F, int... Is>
result_of_transform_t<A, B, F> transformTupleHelper(const A &a,
                                                    const B &b,
                                                    const F &f,
                                                    index_sequence<Is...>) {
    return result_of_transform_t<A, B, F>{
      f(std::get<Is>(a), std::get<Is>(b))...};
}

template <template <typename> class F, typename T, int... Is>
result_of_transform_tmpl_t<T, F> transformTupleTmplHelper(
  const T &a, const T &b, index_sequence<Is...>) {
    return result_of_transform_tmpl_t<T, F>{
      F<typename std::tuple_element<Is, T>::type>{}(std::get<Is>(a),
                                                    std::get<Is>(b))...};
}

}  // namespace internal

template <typename Tuple, typename F>
void applyToTuple(Tuple &tuple, const F &f) {
    internal::applyToTupleHelper(
      tuple, f, make_index_sequence<std::tuple_size<Tuple>::value>{});
};


template <typename... T, typename F>
struct result_of_transform<std::tuple<T...>, F> {
    using type = std::tuple<decltype(std::declval<F>()(std::declval<T>()))...>;
};

template <typename... A, typename... B, typename F>
struct result_of_transform<std::tuple<A...>, std::tuple<B...>, F> {
    using type = std::tuple<decltype(
      std::declval<F>()(std::declval<A>(), std::declval<B>()))...>;
};

template <typename Tuple, typename F>
result_of_transform_t<Tuple, F> transformTuple(const Tuple &tuple, const F &f) {
    return internal::transformTupleHelper(
      tuple, f, make_index_sequence<std::tuple_size<Tuple>::value>{});
};

template <typename A, typename B, typename F>
result_of_transform_t<A, B, F> transformTuple(const A &tuple_a,
                                              const B &tuple_b,
                                              const F &f) {
    return internal::transformTupleHelper(
      tuple_a, tuple_b, f, make_index_sequence<std::tuple_size<A>::value>{});
};

/* The return type of `transformTuple` with arguments of the given type */
template <typename... T, template <typename> class F>
struct result_of_transform_tmpl<std::tuple<T...>, F> {
    using type = std::tuple<decltype(
      std::declval<F<T>>()(std::declval<T>(), std::declval<T>()))...>;
};


template <template <typename> class F, typename T>
result_of_transform_tmpl_t<T, F> transformTupleTmpl(const T &tuple_a,
                                                    const T &tuple_b) {
    return internal::transformTupleTmplHelper<F>(
      tuple_a, tuple_b, make_index_sequence<std::tuple_size<T>::value>{});
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
