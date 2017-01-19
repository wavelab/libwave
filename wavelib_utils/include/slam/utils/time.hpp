#ifndef __SLAM_UTILS_TIME_HPP__
#define __SLAM_UTILS_TIME_HPP__


namespace slam {

// FUNCTIONS
void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

} // end of slam namespace
#endif
