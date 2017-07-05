#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_DEFINE_COMPOSED_VALUE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_DEFINE_COMPOSED_VALUE_HPP

#include <boost/preprocessor.hpp>

// Macros used to wrap each element in a sequence in parens
#define WAVE_IMPL_PAREN_FILLER_0(X, Y) ((X, Y)) WAVE_IMPL_PAREN_FILLER_1
#define WAVE_IMPL_PAREN_FILLER_1(X, Y) ((X, Y)) WAVE_IMPL_PAREN_FILLER_0
#define WAVE_IMPL_PAREN_FILLER_0_END
#define WAVE_IMPL_PAREN_FILLER_1_END
#define WAVE_IMPL_WRAP(seq) BOOST_PP_CAT(WAVE_IMPL_PAREN_FILLER_0 seq, _END)

#define WAVE_IMPL_GET_TYPE(tuple) BOOST_PP_TUPLE_ELEM(2, 0, tuple)
#define WAVE_IMPL_GET_NAME(tuple) BOOST_PP_TUPLE_ELEM(2, 1, tuple)

#define WAVE_IMPL_GET_TYPE_LIST_ITER(r, n, i, tuple) \
    WAVE_IMPL_GET_TYPE(tuple)                        \
    BOOST_PP_COMMA_IF(BOOST_PP_LESS(i, BOOST_PP_DEC(n)))

#define WAVE_IMPL_GET_TYPE_LIST(attributes) \
    BOOST_PP_SEQ_FOR_EACH_I(                \
      WAVE_IMPL_GET_TYPE_LIST_ITER, BOOST_PP_SEQ_SIZE(attributes), attributes)

#define WAVE_IMPL_DEFINE_MEMBER(r, _, i, tuple)                        \
    Ref<WAVE_IMPL_GET_TYPE(tuple) < T, O>> WAVE_IMPL_GET_NAME(tuple) = \
      this->template blockAtIndex<i>();

#define WAVE_IMPL_DEFINE_MEMBERS(attributes) \
    BOOST_PP_SEQ_FOR_EACH_I(WAVE_IMPL_DEFINE_MEMBER, ~, attributes)


#define WAVE_DEFINE_COMPOSED_VALUE_IMPL_IMPL(name, attributes)                 \
    template <typename T, typename O = void>                                   \
    struct name                                                                \
      : public wave::ComposedValue<name<T, O>,                                 \
                                   WAVE_IMPL_GET_TYPE_LIST(attributes)> {      \
        using Base = wave::ComposedValue<name<T, O>,                           \
                                         WAVE_IMPL_GET_TYPE_LIST(attributes)>; \
        using Base::Base;                                                      \
        using Base::operator=;                                                 \
        name() = default;                                                      \
        name(const name &rhs) : Base{rhs} {}                                   \
        WAVE_IMPL_DEFINE_MEMBERS(attributes)                                   \
    }

#define WAVE_DEFINE_COMPOSED_VALUE_IMPL(name, attributes) \
    WAVE_DEFINE_COMPOSED_VALUE_IMPL_IMPL(name, WAVE_IMPL_WRAP(attributes))

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_DEFINE_COMPOSED_VALUE_HPP
