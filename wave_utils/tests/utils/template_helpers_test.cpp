#include "wave/wave_test.hpp"
#include "wave/utils/template_helpers.hpp"

/**
 * These tests demonstrate and check the template helpers.
 *
 * They are sorted into structs only to organize the checks. Most of these
 * checks are done at compile time, with no test that could fail at runtime.
 */
namespace wave {
namespace tmp {

struct Test_index_sequence {
    static_assert(std::is_same<index_sequence<0, 1, 2>,
                               typename make_index_sequence<3>::type>::value,
                  "");
    static_assert(std::is_same<index_sequence<5, 6, 7, 8>,
                               typename make_index_sequence<4, 5>::type>::value,
                  "");
    static_assert(
      std::is_same<index_sequence<-3, -2>,
                   typename make_index_sequence<2, -3>::type>::value,
      "");
};

struct Test_concat_index_sequence {
    using A = index_sequence<0, 1, 2>;
    using B = index_sequence<25, 30>;
    using C = index_sequence<-1>;

    static_assert(
      std::is_same<index_sequence<0, 1, 2, 25, 30, -1>,
                   typename concat_index_sequence<A, B, C>::type>::value,
      "");
    static_assert(std::is_same<index_sequence<-1>,
                               typename concat_index_sequence<C>::type>::value,
                  "");
};

struct Test_sum {
    static_assert(sum<0>::value == 0, "");
    static_assert(sum<1, 2, 3>::value == 6, "");
    static_assert(sum<2, -20, 34, 5>::value == 21, "");
};

struct Test_sum_index_sequence {
    using seq = index_sequence<2, -20, 34, 5>;
    static_assert(sum_index_sequence<seq>::value == 21, "");
};

struct Test_tuple_cat_result {
    using A = std::tuple<int, char, bool>;
    using B = std::tuple<float *, int>;

    static_assert(std::is_same<decltype(std::tuple_cat(A{}, B{})),
                               tuple_cat_result<A, B>>::value,
                  "");
    static_assert(std::is_same<std::tuple<int, char, bool, float *, int>,
                               tuple_cat_result<A, B>>::value,
                  "");
};

struct Test_check_all {
    using seq1 = type_sequence<int, float *, double>;
    using seq2 = type_sequence<int *, float *, double *>;
    static_assert(!check_all<seq1, std::is_pointer>::value, "");
    static_assert(check_all<seq2, std::is_pointer>::value, "");
};

struct Test_clean_method_t {
    struct Foo {
        bool f(int, int);
    };

    static_assert(
      std::is_same<bool(int, int),
                   typename clean_method<decltype(&Foo::f)>::type>::value,
      "");
};

}  // namespace tmp

// Runtime tests

TEST(TemplateHelpers, vectorFromSequence) {
    auto seq = tmp::make_index_sequence<4>();
    auto vec = tmp::vectorFromSequence(seq);
    auto expected = std::vector<int>{0, 1, 2, 3};

    EXPECT_EQ(expected, vec);
};

// sample functor for the next test
struct DoubleInPlace {
    template <typename T>
    void operator()(T &t) const {
        t = 2 * t;
    }
};

TEST(TemplateHelpers, applyToTuple) {
    auto t = std::tuple<double, unsigned, float>{2.5, 1u, 2.9f};

    tmp::applyToTuple(t, DoubleInPlace{});
    EXPECT_DOUBLE_EQ(2 * 2.5, std::get<0>(t));
    EXPECT_EQ(2u, std::get<1>(t));
    EXPECT_FLOAT_EQ(2 * 2.9f, std::get<2>(t));
};

// sample functor for the next test
struct Double {
    template <typename T>
    T operator()(const T &t) const {
        return 2 * t;
    }
};

TEST(TemplateHelpers, transformTuple) {
    const auto t = std::tuple<double, unsigned, float>{2.5, 1u, 2.9f};

    auto res = tmp::transformTuple(t, Double{});
    EXPECT_DOUBLE_EQ(2 * 2.5, std::get<0>(res));
    EXPECT_EQ(2u, std::get<1>(res));
    EXPECT_FLOAT_EQ(2 * 2.9f, std::get<2>(res));
};

// sample functor for the next test
struct AddPointer {
    template <typename T>
    const T *operator()(const T &t) const {
        return &t;
    }
};

TEST(TemplateHelpers, transformTupleChangingType) {
    const auto t = std::tuple<double, float>{2.5, 2.9f};

    auto res = tmp::transformTuple(t, AddPointer{});
    EXPECT_EQ(&std::get<0>(t), std::get<0>(res));
    EXPECT_EQ(&std::get<1>(t), std::get<1>(res));
};

// sample functor for the next test
struct Divide {
    template <typename T>
    T operator()(const T &a, const T &b) const {
        return a / b;
    }
};

TEST(TemplateHelpers, transformTupleBinary) {
    const auto a = std::tuple<double, unsigned, float>{2.5, 10u, 2.9f};
    const auto b = std::tuple<double, unsigned, float>{3.7, 3u, -1.2f};

    auto res = tmp::transformTuple(a, b, Divide{});
    EXPECT_DOUBLE_EQ(std::get<0>(a) / std::get<0>(b), std::get<0>(res));
    EXPECT_EQ(std::get<1>(a) / std::get<1>(b), std::get<1>(res));
    EXPECT_FLOAT_EQ(std::get<2>(a) / std::get<2>(b), std::get<2>(res));
};

TEST(TemplateHelpers, transformTupleTmplWithBinaryFunctorTemplate) {
    const auto a = std::tuple<double, unsigned, float>{2.5, 10u, 2.9f};
    const auto b = std::tuple<double, unsigned, float>{3.7, 3u, -1.2f};

    auto res = tmp::transformTupleTmpl<std::divides>(a, b);
    EXPECT_DOUBLE_EQ(std::get<0>(a) / std::get<0>(b), std::get<0>(res));
    EXPECT_EQ(std::get<1>(a) / std::get<1>(b), std::get<1>(res));
    EXPECT_FLOAT_EQ(std::get<2>(a) / std::get<2>(b), std::get<2>(res));
};

}  // namespace wave
