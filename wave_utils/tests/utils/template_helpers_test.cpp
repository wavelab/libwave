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

}  // namespace tmp

// Runtime tests

TEST(TemplateHelpers, vectorFromSequence) {
    auto seq = tmp::make_index_sequence<4>();
    auto vec = tmp::vectorFromSequence(seq);
    auto expected = std::vector<int>{0, 1, 2, 3};

    EXPECT_EQ(expected, vec);
};

}  // namespace wave
