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

struct BRISKParams {
    int threshold;
    int octaves;
    float patternScale;
};

struct BRISKParamsCustomPattern {
    std::vector<float> radiusList;
    std::vector<int> numberList;
    float dMax;
    float dMin;
    std::vector<int> indexChange;
};

class BRISKDescriptor {
    /** Constructs a BRISKDescriptor Extractor using the default parameters.
     *
     *  @param use_custom_params. If true, loads the default
     *  BRISKParamsCustomPattern configuration, else loads the default
     *  BRISKParams configuration.
     *
     */
    BRISKDescriptor(const bool use_custom_pattern_config);

    /** Constructs a BRISKDescriptor Extractor using parameters specified by the
     *  user.
     *
     *  @param config contains the desired parameter values for a BRISKParams
     *  implementation.
     */
    BRISKDescriptor(const BRISKParamsStandard &config);

    /** Constructs a BRISKDescriptor Extractor using parameters specified by the
     *  user.
     *
     *  @param config contains the desired parameter values for a
     *  BRISKParamsCustomPattern implementation.
     */
    BRISKDescriptor(const BRISKParamsCustom &config);

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
