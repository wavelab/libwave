#include "wave/wave_test.hpp"
#include "wave/utils/template_helpers.hpp"

/**
 * These tests demonstrate and check the template helpers.
 *
 * The only reason for defining functions is to organize the checks. The checks
 * are done at compile time, and the functions are never called.
 */
namespace wave {
namespace tmp {

void test_make_index_sequence() {
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

void test_concat_index_sequence() {
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

void test_sum() {
    static_assert(sum<0>::value == 0, "");
    static_assert(sum<1, 2, 3>::value == 6, "");
    static_assert(sum<2, -20, 34, 5>::value == 21, "");
};

void test_sum_index_sequence() {
    using seq = index_sequence<2, -20, 34, 5>;
    static_assert(sum_index_sequence<seq>::value == 21, "");
};

void test_array_sum() {
    static constexpr std::array<int, 4> array{{2, -20, 34, 5}};
    static_assert(21 == tmp::array_sum(array), "");
};

void test_cumulative_array() {
    constexpr std::array<int, 5> in{{0, 1, 3, 2, 10}};
    constexpr std::array<int, 5> expected{{0, 0, 1, 4, 6}};
    constexpr auto res = tmp::cumulative_array(in);
    static_assert(expected[0] == res[0], "");
    static_assert(expected[1] == res[1], "");
    static_assert(expected[2] == res[2], "");
    static_assert(expected[3] == res[3], "");
    static_assert(expected[4] == res[4], "");
};

}  // namespace tmp
}  // namespace wave
