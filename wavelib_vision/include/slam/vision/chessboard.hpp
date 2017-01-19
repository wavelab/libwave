#ifndef __SLAM_VISION_CHESSBOARD_HPP__
#define __SLAM_VISION_CHESSBOARD_HPP__

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>

#include "slam/utils/utils.hpp"
#include "slam/utils/logging.hpp"


namespace slam {

class Chessboard
{
public:
    bool configured;

    int nb_corners_rows;
    int nb_corners_columns;
    int nb_corners_total;
    cv::Size board_size;

    Chessboard(void);
    int configure(int nb_corners_columns, int nb_corners_rows);
};

} // end of slam namespace
#endif
