/**
 * @file
 * Brute force matcher implementation, derived from descriptor matcher base.
 * @ingroup vision
 */
#ifndef WAVE_VISION_TRACKER_HPP
#define WAVE_VISION_TRACKER_HPP

#include <algorithm>
#include <string>
#include <vector>

#include <boost/bimap.hpp>
#include <opencv2/opencv.hpp>

#include "wave/vision/utils.hpp"
#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

struct FeatureTrack {
    /** The assigned ID corresponding to this feature */
    size_t id;
    /** The pixel location of the feature in the sequence of images */
    std::vector<Vec2> measurement;
    /** The first image the feature is seen in */
    size_t first_image;
    /** The last image the feature is seen in*/
    size_t last_image;
};

template <typename TDetector, typename TDescriptor, typename TMatcher>
class Tracker {
 public:
    /** Default constructor
     *
     * @param detector detector object (FAST, ORB, etc...)
     * @param descriptor descriptor object (BRISK, ORB, etc...)
     * @param matcher matcher object (BruteForceMatcher, FLANN)
     */
    Tracker(TDetector detector, TDescriptor descriptor, TMatcher matcher)
        : detector(detector), descriptor(descriptor), matcher(matcher) {}

    /** Loads series of images into Tracker object for offline tracking.
     *
     * @param new_images the images to be loaded.
     */
    void addImageStream(const std::vector<cv::Mat> new_images) {
        this->images = new_images;
    };

    /** Offline method of image tracking, using list of images already loaded.
     */
    std::vector<std::vector<FeatureTrack>> offlineTracker();

 private:
    size_t generateFeatureID() {
        static size_t id = 0;
        return id++;
    }

    void detectAndCompute(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptor);

    std::vector<cv::Mat> images;
    TDetector detector;
    TDescriptor descriptor;
    TMatcher matcher;
};
}  // namespace wave

#endif  // WAVE_VISION_TRACKER_HPP
