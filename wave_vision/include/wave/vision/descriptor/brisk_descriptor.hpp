/**
 * @file
 * BRISK Descriptor Extractor implementation, derived from Descriptor Extractor
 * base class.
 * @ingroup vision
 */
#ifndef WAVE_BRISK_DESCRIPTOR_HPP
#define WAVE_BRISK_DESCRIPTOR_HPP

/** The wave namespace */
namespace wave {

struct BRISKDescriptorParams {
    std::vector<float> radiusList;
    std::vector<int> numberList;
    float dMax;
    float dMin;
    std::vector<int> indexChange;
};

class BRISKDescriptor {
    /** Constructs a BRISKDescriptor Extractor using the default parameters. */
    BRISKDescriptor();

    /** Constructs a BRISKDescriptor Extractor using parameters specified by the
     *  user.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation.
     */
    BRISKDescriptor(const BRISKDescriptorParams &config);

    /** Constructs a BRISKDescriptor Extractor using parameters specified by the
     *  user.
     *
     *  @param config contains the desired parameter values for a
     *  BRISKParamsCustomPattern implementation.
     */
    BRISKDescriptor(const BRISKDescriptorParams &config);

    /** Constructs a BRISKDescriptor Extractor using parameters found in the
    *   linked .yaml file.
    *
    *   @param config_path is the path to a .yaml file, containing the desired
    *   parameters for the BRISK Descriptor Extractor. The .yaml file can be
    *   configured for both BRISKParams or BRISKParamsCustomPattern.
    */
    BRISKDescriptor(const std::string &config_path);
};
} /** end of namespace wave */

#endif  // WAVE_BRISK_DESCRIPTOR_HPP
