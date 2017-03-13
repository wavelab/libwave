#ifndef __WAVE_UTILS_FILESYSTEM_HPP__
#define __WAVE_UTILS_FILESYSTEM_HPP__

#include <stdio.h>

#include <string>
#include <vector>
#include <numeric>
#include <iostream>


namespace wave {

bool file_exists(const std::string &name);
std::vector<std::string> path_split(const std::string path);
void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out);

}  // end of wave namepsace
#endif
