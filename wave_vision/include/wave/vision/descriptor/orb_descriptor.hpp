/**
 * @file
 * ORB Descriptor Extractor implementation, derived from DescriptorExtractor
 * base class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_ORB_DESCRIPTOR_HPP
#define WAVE_VISION_ORB_DESCRIPTOR_HPP

#include <string>
#include <vector>

#include "wave/vision/descriptor/descriptor_extractor.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Configuration parameters for the ORB descriptor extractor.
 *
 *  The full description of each parameter can be found
 *  [here][ORBDescriptorParams]. Note: the ORBDescriptorParams struct only
 *  consists of the parameters required for feature description, not detection.
 *  ORB Feature Detector parameters can be found under ORBDetectorParams.
 *
 *  [ORBDescriptorParams]:
 *  http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#adc371099dc902a9674bd98936e79739c
 */
struct ORBDescriptorParams {
    ORBDescriptorParams() = default;

    ORBDescriptorParams(const int &tuple_size, const int &patch_size)
        : tuple_size(tuple_size), patch_size(patch_size) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     *  @param config_path the path to the location of the configuration file.
     */
    explicit ORBDescriptorParams(const std::string &config_path);

    /** The number of points that compose the ORB descriptor. A value of 2
     *  constructs the descriptor by comparing the brightnesses of a random pair
     *  of points. Other values are 3 or 4; 3 will use 3 random points, and 4
     *  similarly will use 4 random points. For these last two options, the
     *  output result will be 2 bits, as the result is the index of the
     *  brightest point (0, 1, 2, 3). Thus, the distance method used for the
     *  BruteForceMatcher or FLANNMatcher will have to be cv::NORM_HAMMING_2, a
     *  variation to the Hamming distance, when tuple_size = 3 or 4.
     *
     *  OpenCV refers to this as "WTA_K." tuple_size is the description from
     *  Kaehler and Bradski's book: "Learning OpenCV3: Computer Vision in C++
     *  with the OpenCV Library."
     *
     *  Options: 2, 3, or 4.
     *
     *  Default: 2.
     */
    int tuple_size = 2;

    /** The size of the square patch used in the random point sampling to
     *  construct the descriptor. This patch is smoothed using an integral
     *  image.
     *
     *  Default: 31 (31 x 31 pixels). Must be greater than 0.
     */
    int patch_size = 31;
};

/** Representation of a descriptor extractor using the ORB algorithm.
 *
 *  Internally, this class is wrapping OpenCV's ORB descriptor module.
 *  Further reference on ORB can be found [here][opencv_orb_descriptor].
 *
 *  [opencv_orb_descriptor]:
 *  http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html
 */
class ORBDescriptor : public DescriptorExtractor {
 public:
    explicit ORBDescriptor(
      const ORBDescriptorParams &config = ORBDescriptorParams{});

    /** Reconfigures the cv::ORB object with new values requested by the user.
     *
     * @param new_config containing the desired configuration values.
     */
    void configure(const ORBDescriptorParams &new_config);

    /** Returns the current configuration parameters being used by the ORB
     *  Descriptor Extractor
     *
     *  @return the current configuration values.
     */
    ORBDescriptorParams getConfiguration() const;

    /** Extracts descriptors from the keypoints in an image, using the ORB
     *  descriptor extractor.
     *
     *  @param image the image to detect features in.
     *  @param keypoints the keypoints from the detected image
     *
     *  @return an array containing the computed descriptors.
     */
    cv::Mat extractDescriptors(const cv::Mat &image,
                               std::vector<cv::KeyPoint> &keypoints);

 private:
    /** The pointer to the wrapped cv::ORB object. */
    cv::Ptr<cv::ORB> orb_descriptor;

    /** Checks whether the desired configuration is valid.
     *
     * @param check_config the desired configuration values.
     */
    void checkConfiguration(const ORBDescriptorParams &check_config);
};
}  // namespace wave

#endif  // WAVE_VISION_ORB_DESCRIPTOR_HPP
