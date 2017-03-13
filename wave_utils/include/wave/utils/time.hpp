#ifndef __WAVE_UTILS_TIME_HPP__
#define __WAVE_UTILS_TIME_HPP__

#include <time.h>
#include <sys/time.h>

namespace wave {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now(void);

}  // end of wave namespace
#endif
