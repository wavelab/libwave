#ifndef WAVE_UTILS_TIME_HPP
#define WAVE_UTILS_TIME_HPP

#include <time.h>
#include <sys/time.h>

namespace wave {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
double time_now(void);

}  // end of wave namespace
#endif
