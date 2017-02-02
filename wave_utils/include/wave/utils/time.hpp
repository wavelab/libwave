#ifndef __wave_UTILS_TIME_HPP__
#define __wave_UTILS_TIME_HPP__


namespace wave {

// FUNCTIONS
void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

}  // end of wave namespace
#endif
