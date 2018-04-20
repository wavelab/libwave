/**
 * @file
 * Feature tracker implementation for a stereo camera setup.
 * @ingroup vision
 */

#ifndef WAVE_STEREO_TRACKER_HPP
#define WAVE_STEREO_TRACKER_HPP

#include "wave/vision/tracker/tracker.hpp"

namespace wave {

template <typename TDetector, typename TDescriptor, typename TMatcher>
class StereoTracker {
 public:
    StereoTracker(TDetector detector, TDescriptor descriptor, TMatcher matcher)
        : detector(detector), descriptor(descriptor), matcher(matcher) {}

    ~StereoTracker() = default;

 public:
 private:
    TDetector detector;

    TDescriptor descriptor;

    TMatcher matcher;

 private:
};

}  // namespace wave

#endif  // WAVE_STEREO_TRACKER_HPP
