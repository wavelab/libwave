#ifndef __wavelib_UTILS_UTILS_HPP__
#define __wavelib_UTILS_UTILS_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <iostream>

#include "wavelib/utils/data.hpp"
#include "wavelib/utils/math.hpp"
#include "wavelib/utils/time.hpp"
#include "wavelib/utils/logging.hpp"


namespace wavelib {

// MACROS
#define UNUSED(expr) do { (void)(expr); } while (0)


// FUNCTIONS
void rmtrailslash(std::string &path);

} // end of wavelib namespace
#endif
