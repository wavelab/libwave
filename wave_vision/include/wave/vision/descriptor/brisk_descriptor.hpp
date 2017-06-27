/**
 * @file
 * BRISK Descriptor Extractor implementation, derived from Descriptor Extractor
 * base class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_BRISK_DESCRIPTOR_HPP
#define WAVE_VISION_BRISK_DESCRIPTOR_HPP

#include <string>
#include <vector>

#include "wave/vision/descriptor/descriptor_extractor.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Configuration parameters for the BRISK descriptor extractor.
 *
 *  The BRISK Descriptor creates a pattern of points organized as concentric
 *  circles around the keypoint. Each point in these rings are then used for
 *  brightness comparison tests to construct the descriptor.
 */
struct BRISKDescriptorParams {
    /** Default Constructor */
    BRISKDescriptorParams() {}

    BRISKDescriptorParams(std::vector<float> rlist,
                          std::vector<int> nlist,
                          float d_max,
                          float d_min)
        : radius_list(rlist), number_list(nlist), d_max(d_max), d_min(d_min) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     *  @param config_path the path to the location of the configuration file
     */
    BRISKDescriptorParams(const std::string &config_path) {
        // Extract parameters from .yaml file.
        ConfigParser parser;

        std::vector<float> radius_list;
        std::vector<int> number_list;
        float d_max;
        float d_min;

        // Add parameters to parser, to be loaded. If path cannot be found,
        // throw an exception
        parser.addParam("radius_list", &radius_list);
        parser.addParam("number_list", &number_list);
        parser.addParam("d_max", &d_max);
        parser.addParam("d_min", &d_min);

        if (parser.load(config_path) != 0) {
            throw std::invalid_argument(
              "Failed to Load BRISKDescriptor Configuration");
        }

        this->radius_list = radius_list;
        this->number_list = number_list;
        this->d_max = d_max;
        this->d_min = d_min;
    }

    /** radius_list defines the radius of each subsequent circle (in pixels).
     *  All numbers must be positive. Cannot be empty.
     *
     *  Recommended: radius_list = {0.0f, 2.47f, 4.17f, 6.29f, 9.18f}
     */
    std::vector<float> radius_list = {0.0f, 2.47f, 4.17f, 6.29f, 9.18f};

    /** number_list defines the number of points in each circle. Must be the
     *  same size as radiusList. All numbers must be positive. Cannot be empty.
     *
     *  Recommended: number_list = {1, 10, 14, 15, 20};
     */
    std::vector<int> number_list = {1, 10, 14, 15, 20};

    /** d_max and d_min are threshold distances to classify a pair of points as
     *  a \a long pair or a \a short pair. Short pairs are not used in the
     *  brightness comparison, due to balancing effects of local gradients. The
     *  long pairs are not used in the assembly of the bit vector descriptor.
     *
     *  d_max specifies the maximum threshold for short pairs. The value of
     *  d_max must be less than that of d_min.
     *
     *  Recommended: d_max = 5.85f
     */
    float d_max = 5.85f;

    /** d_min specifies the minimum threshold for long pairs.
     *
     *  The value of d_min must be more than that of d_max.
     *
     *  Recommended:  d_min = 8.2f
     */
    float d_min = 8.2f;

    /** OpenCV refers to this as a parameter for "index remapping of the bits."
     *  Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
     *  with the OpenCV Library" states this parameter is unused, and should be
     *  omitted.
     */
    std::vector<int> index_change;
};

class BRISKDescriptor : public DescriptorExtractor {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation. Uses default values if not specified.
     */
    explicit BRISKDescriptor(
      const BRISKDescriptorParams &config = BRISKDescriptorParams{});

    /** Returns the current configuration parameters being used by the
     *  BRISK Descriptor Extractor.
     *
     *  @return a struct containing the current configuration values.
     */
    BRISKDescriptorParams getConfiguration() const;

    /** Extracts descriptors from the keypoints in an image, using the BRISK
     *  extractor.
     *
     *  @param image the image to detect features in.
     *  @param keypoints the keypoints from the detected image
     *
     *  @return an array containing the computed descriptors.
     */
    cv::Mat extractDescriptors(cv::Mat &image,
                               std::vector<cv::KeyPoint> &keypoints);

 private:
    /** The pointer to the wrapped cv::BRISK object. */
    cv::Ptr<cv::BRISK> brisk_descriptor;

    /** The current configuration */
    BRISKDescriptorParams current_config;

    /** Checks whether the desired configuration is valid.
     *
     *  @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const BRISKDescriptorParams &check_config);
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_BRISK_DESCRIPTOR_HPP
