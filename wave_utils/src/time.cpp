#include "wave/utils/time.hpp"


namespace wave {

void tic(struct timespec *tic) {
    clock_gettime(CLOCK_MONOTONIC, tic);
}

float toc(struct timespec *tic) {
    struct timespec toc;
    float time_elasped;

    clock_gettime(CLOCK_MONOTONIC, &toc);
    time_elasped = (toc.tv_sec - tic->tv_sec);
    time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

    return time_elasped;
}

float mtoc(struct timespec *tic) {
    return toc(tic) * 1000.0;
}

double time_now(void) {
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

}  // namespace wave
