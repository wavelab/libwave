#ifndef WAVE_UTILS_LOGGING_HPP
#define WAVE_UTILS_LOGGING_HPP

namespace wave {

#define FILENAME \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...) \
    fprintf(              \
      stderr, "[ERROR] [%s:%d] " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

}  // end of wave namespace
#endif
