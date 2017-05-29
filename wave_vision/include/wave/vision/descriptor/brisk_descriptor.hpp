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

/**
 *  BRISKDescriptorParams struct contains the configuration parameters for the
 *  BRISKDescriptor extractor.
 *
 */
struct BRISKDescriptorParams {
    std::vector<float> radiusList;
    std::vector<int> numberList;
    float dMax;
    float dMin;
    std::vector<int> indexChange;
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

    /** Reconfigures the BRISKDescriptor object with new values requested by
     *  the user
     *
     *  @param new_config, containing the desired configuration values.
     */
    void configure(const BRISKDescriptorParams &new_config);

    /** Returns the current configuration parameters being used by the
     *  BRISKDescriptor Extractor.
     *
     *  @return a struct containing the current configuration values.
     */
    BRISKDescriptorParams getConfiguration();

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
    /** The pointer to the wrapped cv::BRISK object */
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
