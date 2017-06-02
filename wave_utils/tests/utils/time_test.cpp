#include "wave/wave_test.hpp"
#include "wave/utils/time.hpp"


namespace wave {

TEST(Utils_time, ticAndtoc) {
    struct timespec start;

    tic(&start);
    usleep(10 * 1000);
    ASSERT_TRUE(toc(&start) < 0.011);
    ASSERT_TRUE(toc(&start) > 0.009);
    ASSERT_TRUE(mtoc(&start) < 11.0);
    ASSERT_TRUE(mtoc(&start) > 9.0);
}

}  // namespace wave
