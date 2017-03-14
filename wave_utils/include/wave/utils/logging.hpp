#ifndef __WAVE_UTILS_LOGGING_HPP__
#define __WAVE_UTILS_LOGGING_HPP__

#define __FILENAME__ \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define log_err(M, ...)                \
    fprintf(stderr,                    \
            "[ERROR] [%s:%d] " M "\n", \
            __FILENAME__,              \
            __LINE__,                  \
            ##__VA_ARGS__)

#define log_info(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

#endif
