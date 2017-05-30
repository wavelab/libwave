/**
 * @file
 * BRISK Descriptor Extractor implementation, derived from Descriptor Extractor
 * base class.
 * @ingroup vision
 */
#ifndef WAVE_BRISK_DESCRIPTOR_HPP
#define WAVE_BRISK_DESCRIPTOR_HPP

/** C++ Headers */
#include <string>
#include <vector>

/** Libwave Headers */
#include "wave/vision/descriptor/descriptor_extractor.hpp"

/** The wave namespace */
namespace wave {
/** @addtogroup vision
 *  @{ */

/** BRISKDescriptorParams struct contains the configuration parameters for the
 *  BRISKDescriptor extractor.
 *
 *  The BRISK Descriptor creates a pattern of points organized as concentric
 *  circles around the keypoint. Each point in these rings are then used for
 *  brightness comparison tests to construct the descriptor.
 */
struct BRISKDescriptorParams {
    /** radiusList defines the radius of each subsequent circle (in pixels). All
     *  numbers must be positive. Cannot be empty.
     *
     *  Default: radiusList = {0.0f, 2.47f, 4.17f, 6.29f, 9.18f}
     */
    std::vector<float> radiusList;

    /** numberList defines the number of points in each circle. Must be the same
     * size as radiusList. All numbers must be positive. Cannot be empty.
     *
     * Default: numberList = {1, 10, 14, 15, 20};
     */
    std::vector<int> numberList;

    /** dMax and dMin are threshold distances to classify a pair of points as a
     *  \a long pair or a \a short pair. Short pairs are not used in the
     *  brightness comparison, due to balancing effects of local gradients. The
     *  long pairs are not used in the assembly of the bit vector descriptor.
     *  The value of dMax must be less than that of dMin.
     *
     *  Default: dMax = 5.85f, dMin = 8.2f.
     */
    float dMax;
    float dMin;
};

class BRISKDescriptor : public DescriptorExtractor {
    /** Constructs a BRISKDescriptor Extractor using the default parameters. */
    BRISKDescriptor();

    /** Constructs a BRISKDescriptor Extractor using parameters specified by the
     *  user.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation.
     */
    explicit BRISKDescriptor(const BRISKDescriptorParams &config);

    /** Constructs a BRISKDescriptor Extractor using parameters found in the
    *   linked .yaml file.
    *
    *   @param config_path is the path to a .yaml file, containing the desired
    *   parameters for the BRISK Descriptor Extractor. The .yaml file can be
    *   configured for both BRISKParams or BRISKParamsCustomPattern.
    */
    BRISKDescriptor(const std::string &config_path);

    /** Destructor */
    ~BRISKDescriptor();

    /** Extracts descriptors from the keypoints in an image, using the BRISK
     *  extractor.
     *
     *  @param image, the image to detect features in.
     *  @param keypoints, the keypoints from the detected image
     *  @return an array containing the computed descriptors.
     */
    cv::Mat extractDescriptors(const cv::Mat &image,
                               const std::vector<cv::KeyPoint> &keypoints);

 private:
    /** The pointer to the wrapped cv::BRISK object. */
    cv::Ptr<cv::BRISK> brisk_descriptor;

    /** Checks whether the desired configuration is valid.
     *
     *  @param check_config, containing the desired configuration values.
     */
    void checkConfiguration(const BRISKDescriptorParams &check_config);
};

/** @} end of group */
} /** end of namespace wave */

#endif  // WAVE_BRISK_DESCRIPTOR_HPP
