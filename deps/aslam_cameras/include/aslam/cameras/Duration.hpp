#ifndef ASLAM_CAMERA_DURATION_HPP
#define ASLAM_CAMERA_DURATION_HPP

/* aslam_cameras used a custom "Duration" class (for durations related to
 * rolling shutters). This file contains definitions to convert that to
 * std::chrono::duration.
 */

#include <chrono>

namespace aslam {
    // A duration storing seconds as a double
    using Duration = std::chrono::duration<double>;

}  // namespace aslam

#endif  // ASLAM_CAMERA_DURATION_HPP
