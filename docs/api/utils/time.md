# wave/utils/time.hpp

This module contains timing functions, useful for measuring how long a particular function executed etc.


## Functions

    void tic(struct timespec *t);
    float toc(struct timespec *t);
    float mtoc(struct timespec *t);

Similar to how matlab's tic and toc work, `tic()` starts the timer where the variable `t` stores the start time, when `toc` is called it returns the time difference in seconds. `mtoc()` returns the time difference in milli-seconds.

---

    double time_now(void);

Returns the time since epoch in seconds.
