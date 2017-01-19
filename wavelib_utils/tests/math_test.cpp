#include <gtest/gtest.h>

#include "slam/utils/math.hpp"


TEST(kronecker_product, test)
{
    slam::MatX A(2, 2);
    slam::MatX B(2, 2);
    slam::MatX expected(4, 4);
    slam::MatX product;

    // setup
    A << 1, 2,
         3, 4;
    B << 0, 5,
         6, 7;
    expected << 0, 5, 0, 10,
              6, 7, 12, 14,
              0, 15, 0, 20,
              18, 21, 24, 28;

    // test and assert
    product = slam::kronecker_product(A, B);
    std::cout << product << std::endl;
    ASSERT_EQ(expected, product);
    ASSERT_EQ(4, product.rows());
    ASSERT_EQ(4, product.cols());
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
