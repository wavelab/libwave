/** @file
 * @ingroup utils
 *
 * File system functions that C++ did not come with.
 */

#ifndef WAVE_UTILS_FILE_HPP
#define WAVE_UTILS_FILE_HPP

#include <stdio.h>
#include <cstring>
#include <dirent.h>

#include <string>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>

#include "wave/utils/log.hpp"


namespace wave {
/** @addtogroup utils
 *  @{ */

/** Remove directory at `path` */
int remove_dir(const std::string &path);

/** Checks if a file exists at the given path */
bool file_exists(const std::string &file_name);

/** Splits `path` with the `/` token.
 * @return vector of separated path elements */
std::vector<std::string> path_split(const std::string path);

/** Combines `path1` with `path2`. The result is written to `out`.
 *
 * This function is modeled after python's `os.path.join` function. Example
 * usage:
 * ```
 * paths_combine("/a/b", "c/d", out) --> out = "/a/b/c/d"
 * paths_combine("/a/b", "../", out) --> out = "/a"
 * paths_combine("/a/b/c", "../..", out) --> out = "/a"
 * ```
 */
void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out);

/** @} group utils */
}  // namespace wave

#endif  // WAVE_UTILS_FILESYSTEM_HPP
