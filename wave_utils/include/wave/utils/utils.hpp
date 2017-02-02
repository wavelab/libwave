#ifndef __wave_UTILS_UTILS_HPP__
#define __wave_UTILS_UTILS_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <iostream>

#include "wave/utils/data.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/time.hpp"
#include "wave/utils/logging.hpp"


namespace wave {

// MACROS
#define UNUSED(expr) \
  do {               \
    (void) (expr);   \
  } while (0)


// FUNCTIONS
void rmtrailslash(std::string &path);

}  // end of wave namespace
#endif
