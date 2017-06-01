/** @file
 * @ingroup utils
 *
 * File system functions that C++ did not come with.
 */

#ifndef WAVE_UTILS_FILE_HPP
#define WAVE_UTILS_FILE_HPP

#include <stdio.h>
#include <dirent.h>

#include <string>
#include <cerrno>
#include <cstring>
#include <vector>
#include <numeric>
#include <iostream>
#include <sstream>


namespace wave {
/** @addtogroup utils
 *  @{ */

/** Checks if a file exists at the given path */
bool file_exists(const std::string &file_name);

/** Remove directory **/
void remove_dir(const std::string &path);

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

}  // end of wave namepsace
#endif
