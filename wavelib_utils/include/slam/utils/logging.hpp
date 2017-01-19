#ifndef __SLAM_UTILS_LOGGING_HPP__
#define __SLAM_UTILS_LOGGING_HPP__

#include <stdio.h>
#include <iostream>
#include <cstdarg>


namespace slam {

#define ERROR_FORMAT "[ERROR] (%s:%d) "
#define WARN_FORMAT "[WARN] (%s:%d) "
#define INFO_FORMAT "[INFO] "

#define LOG_ERROR(M, ...) \
    fprintf(stderr, ERROR_FORMAT M "\n", __func__, __LINE__, ##__VA_ARGS__)
#define LOG_WARN(M, ... ) \
    fprintf(stderr, WARN_FORMAT M "\n", __func__, __LINE__, ##__VA_ARGS__ )
#define LOG_INFO(M, ... ) \
    fprintf(stderr, INFO_FORMAT M "\n", ##__VA_ARGS__ )

} // end of slam namespace
#endif
