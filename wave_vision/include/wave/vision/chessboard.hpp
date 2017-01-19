#ifndef __wave_VISION_CHESSBOARD_HPP__
#define __wave_VISION_CHESSBOARD_HPP__

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>

#include "wave/utils/utils.hpp"
#include "wave/utils/logging.hpp"


namespace wave {

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

} // end of wave namespace
#endif
