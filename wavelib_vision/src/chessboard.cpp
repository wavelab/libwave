#include "slam/vision/chessboard.hpp"


namespace slam {

Chessboard::Chessboard(void)
{
    this->configured = false;

    this->nb_corners_rows = 0;
    this->nb_corners_columns = 0;
    this->nb_corners_total = 0;
    this->board_size = cv::Size(0, 0);
}

int Chessboard::configure(int nb_corners_columns, int nb_corners_rows)
{
    this->configured = true;

    this->nb_corners_rows = nb_corners_rows;
    this->nb_corners_columns = nb_corners_columns;
    this->nb_corners_total = nb_corners_rows * nb_corners_columns;
    this->board_size = cv::Size(nb_corners_columns, nb_corners_rows);

    return 0;
}

} // end of slam namespace
