#include <gtest/gtest.h>

#include "slam/vision/chessboard.hpp"


TEST(Chessboard, constructor)
{
    slam::Chessboard chessboard;

    ASSERT_EQ(false, chessboard.configured);

    ASSERT_EQ(0, chessboard.nb_corners_rows);
    ASSERT_EQ(0, chessboard.nb_corners_columns);
    ASSERT_EQ(0, chessboard.nb_corners_total);
    ASSERT_EQ(cv::Size(0, 0), chessboard.board_size);
}

TEST(Chessboard, configure)
{
    slam::Chessboard chessboard;

    chessboard.configure(1, 2);

    ASSERT_EQ(true, chessboard.configured);

    ASSERT_EQ(2, chessboard.nb_corners_rows);
    ASSERT_EQ(1, chessboard.nb_corners_columns);
    ASSERT_EQ(2, chessboard.nb_corners_total);
    ASSERT_EQ(cv::Size(1, 2), chessboard.board_size);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
