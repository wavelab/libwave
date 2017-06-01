/** @file
 * @ingroup utils
 *
 * Timing functions, useful for measuring how long a particular function
 * executed, etc.
 */

#ifndef WAVE_UTILS_TIME_HPP
#define WAVE_UTILS_TIME_HPP

#include <time.h>
#include <sys/time.h>

namespace wave {
/** @addtogroup utils
 *  @{ */

/** Similar to how matlab's tic and toc work, `tic()` starts the timer where the
 * variable `t` stores the start time, when `toc()` is called it returns the
 * time
 * difference in seconds. `mtoc()` returns the time difference in milli-seconds.
 */
void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

/** @return the time since epoch in seconds. */
double time_now(void);

/** @} group utils */
}  // namespace wave

#endif  // WAVE_UTILS_TIME_HPP
