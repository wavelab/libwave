#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "slam/utils/math.hpp"
#include "slam/utils/data.hpp"
#include "slam/optimization/optimizers/ransac.hpp"

#define TEST_DATA "tests/data/ransac/ransac_sample.dat"


TEST(RANSAC, constructor)
{
    slam::RANSAC ransac;

    ASSERT_EQ(ransac.configured, false);

    ASSERT_EQ(ransac.max_iter, 0);
    ASSERT_FLOAT_EQ(ransac.thresh_ratio, 1.0);
    ASSERT_FLOAT_EQ(ransac.thresh_dist, 0.0);

    ASSERT_EQ(ransac.iter, 0);
    ASSERT_EQ(ransac.max_inliers, 0);
    ASSERT_FLOAT_EQ(ransac.model_params[0], 0.0);
    ASSERT_FLOAT_EQ(ransac.model_params[1], 0.0);
}

TEST(RANSAC, configure)
{
    slam::RANSAC ransac;

    ransac.configure(10, 0.8, 0.1);

    ASSERT_EQ(ransac.configured, true);

    ASSERT_EQ(ransac.max_iter, 10);
    ASSERT_FLOAT_EQ(ransac.thresh_ratio, 0.8);
    ASSERT_FLOAT_EQ(ransac.thresh_dist, 0.1);

    ASSERT_EQ(ransac.iter, 0);
    ASSERT_EQ(ransac.max_inliers, 0);
    ASSERT_FLOAT_EQ(ransac.model_params[0], 0.0);
    ASSERT_FLOAT_EQ(ransac.model_params[1], 0.0);
}

TEST(RANSAC, RandomSample)
{
    int retval;
    slam::Vec2 sample;
    slam::MatX data(2, 100);
    slam::RANSAC ransac;

    // setup
    for (int i = 0; i < 100; i++) {
        data(0, i) = rand() % 100;
        data(1, i) = rand() % 100;
    }
    ransac.configure(10, 0.8, 0.1);

    // test and assert
    sample << -1, -1;
    retval = ransac.randomSample(data, sample);

    ASSERT_EQ(retval, 0);
    ASSERT_NE(sample(0), -1);
    ASSERT_NE(sample(1), -1);
}

TEST(RANSAC, computeDistances)
{
    int retval;
    slam::MatX data(2, 100);
    slam::Vec2 p1;
    slam::Vec2 p2;
    slam::VecX dists;
    slam::RANSAC ransac;

    // setup
    for (int i = 0; i < 100; i++) {
        data(0, i) = i;
        data(1, i) = i;
    }
    ransac.configure(10, 0.8, 0.1);

    // test and assert
    ransac.randomSample(data, p1);
    ransac.randomSample(data, p2);
    retval = ransac.computeDistances(data, p1, p2, dists);

    ASSERT_EQ(retval, 0);
}

TEST(RANSAC, computeInliers)
{
    int retval;
    slam::RANSAC ransac;
    slam::VecX dists(10);

    // setup
    dists << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
    ransac.configure(10, 0.8, 0.5);

    // test and assert
    retval = ransac.computeInliers(dists);
    ASSERT_EQ(retval, 0);
    ASSERT_EQ(ransac.inliers.size(), 5);
    ASSERT_EQ(ransac.inliers[0], 0);
    ASSERT_EQ(ransac.inliers[1], 1);
    ASSERT_EQ(ransac.inliers[2], 2);
    ASSERT_EQ(ransac.inliers[3], 3);
    ASSERT_EQ(ransac.inliers[4], 4);
}

TEST(RANSAC, update)
{
    int retval;
    slam::RANSAC ransac;
    slam::Vec2 p1;
    slam::Vec2 p2;

    // setup
    ransac.configure(10, 0.8, 0.5);
    ransac.threshold = 2;
    ransac.max_inliers = 3;
    ransac.inliers.push_back(0);
    ransac.inliers.push_back(1);
    ransac.inliers.push_back(2);
    ransac.inliers.push_back(3);
    p1 << 1.0, 2.0;
    p2 << 3.0, 4.0;

    // test and assert
    retval = ransac.update(p1, p2);
    ASSERT_EQ(ransac.max_inliers, 4);
    ASSERT_FLOAT_EQ(ransac.model_params[0], 1.0);
    ASSERT_FLOAT_EQ(ransac.model_params[1], 1.0);
}

TEST(RANSAC, optimize)
{
    slam::MatX data;
    slam::MatX x;
    slam::RANSAC ransac;

    // setup
    slam::csv2mat(TEST_DATA, true, data);
    x = data.transpose();
    ransac.configure(40, 0.5, 5);

    // test and assert
    clock_t begin = clock();
    ransac.optimize(x);
    clock_t end = clock();

    double secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "elasped: " << secs << " seconds" << std::endl;

    ASSERT_TRUE(ransac.model_params[0] < 23);
    ASSERT_TRUE(ransac.model_params[0] > 18);
    ASSERT_TRUE(ransac.model_params[1] < 13);
    ASSERT_TRUE(ransac.model_params[1] > 8);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
