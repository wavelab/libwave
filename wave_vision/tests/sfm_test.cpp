#include <gtest/gtest.h>

#include "wave/utils/utils.hpp"
#include "wave/vision/utils.hpp"
#include "wave/vision/sfm.hpp"

#define TEST_DATA_1 "tests/data/pts1.dat"
#define TEST_DATA_2 "tests/data/pts2.dat"


TEST(SFM, constructor)
{
    wave::SFM sfm;

}

TEST(SFM, configure)
{
    wave::SFM sfm;
    wave::Mat3 K;

    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    sfm.configure(K);
    ASSERT_EQ(true, sfm.configured);
}

TEST(SFM, recoverPose)
{
    wave::SFM sfm;
    wave::SFMPose pose;
    wave::Mat3 K;
    wave::MatX pts1, pts2;

    // setup
    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    wave::csv2mat(TEST_DATA_1, false, pts1);
    wave::csv2mat(TEST_DATA_2, false, pts2);

    // wave::normalize_2dpts(960, 720, pts1);
    // wave::normalize_2dpts(960, 720, pts2);

    sfm.configure(K);
    std::cout << pts1 << std::endl << std::endl;
    std::cout << pts2 << std::endl << std::endl;

    // test
    sfm.recoverPose(pts1, pts2, pose);
    std::cout << pose.R.eulerAngles(0, 1, 2) << std::endl << std::endl;
    std::cout << pose.t << std::endl;
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
