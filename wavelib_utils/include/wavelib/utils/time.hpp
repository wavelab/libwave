#ifndef __wavelib_UTILS_TIME_HPP__
#define __wavelib_UTILS_TIME_HPP__


namespace wavelib {

// FUNCTIONS
void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

} // end of wavelib namespace
#endif
