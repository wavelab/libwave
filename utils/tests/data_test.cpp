#include <gtest/gtest.h>

#include "wavelib/utils/math.hpp"
#include "wavelib/utils/data.hpp"


#define TEST_DATA "tests/data/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"


TEST(csvrows, test)
{
    int rows;

    rows = wavelib::csvrows(TEST_DATA);
    printf("rows: %d\n", rows);
    ASSERT_EQ(281, rows);
}

TEST(csvcols, test)
{
    int cols;

    cols = wavelib::csvcols(TEST_DATA);
    printf("cols: %d\n", cols);
    ASSERT_EQ(2, cols);
}

TEST(csv2mat, test)
{
    wavelib::MatX data;

    wavelib::csv2mat(TEST_DATA, true, data);

    printf("rows: %d\n", (int) data.rows());
    printf("cols: %d\n", (int) data.cols());

    ASSERT_EQ(280, data.rows());
    ASSERT_EQ(2, data.cols());
    ASSERT_FLOAT_EQ(-2.22482078596, data(0, 0));
    ASSERT_FLOAT_EQ(9.9625789766, data(0, 1));
    ASSERT_FLOAT_EQ(47.0485650525, data(279, 0));
    ASSERT_FLOAT_EQ(613.503760567, data(279, 1));
}

TEST(mat2csv, test)
{
    wavelib::MatX x;
    wavelib::MatX y;

    wavelib::csv2mat(TEST_DATA, true, x);
    wavelib::mat2csv(TEST_OUTPUT, x);
    wavelib::csv2mat(TEST_OUTPUT, false, y);

    for (int i = 0; i < x.rows(); i++) {
        for (int j = 0; j < x.cols(); j++) {
            ASSERT_NEAR(x(i, j), y(i, j), 0.1);
        }
    }
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
