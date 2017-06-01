/** @file
 * @ingroup utils
 *
 * Functions to log errors and info to `stderr` and `stdout`.
 *
 * `LOG_ERROR` and `LOG_INFO` both are simple `fprintf()` that can be use to
 * write message `M` to `stderr` and `stdout`. For example:
 * ```
 * LOG_ERROR("Failed to load configuration file [%s]", config_file.c_str());
 * LOG_INFO("Parameter was not found! Loading defaults!");
 * ```
 */

#ifndef WAVE_UTILS_LOG_HPP
#define WAVE_UTILS_LOG_HPP

namespace wave {
/** @addtogroup utils
 *  @{ */

#define FILENAME \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...) \
    fprintf(              \
      stderr, "[ERROR] [%s:%d] " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

/** @} end of group */
}  // end of wave namespace
#endif
