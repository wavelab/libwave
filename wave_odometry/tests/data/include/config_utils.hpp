#ifndef WAVE_CONFIG_UTILS_HPP
#define WAVE_CONFIG_UTILS_HPP

#include "wave/odometry/laser_odom.hpp"
#include "wave/utils/config.hpp"

namespace wave {

void LoadSensorParameters(const std::string &path, const std::string &filename, wave::RangeSensorParams &senparams);

void LoadParameters(const std::string &path, const std::string &filename, wave::LaserOdomParams &params);

void loadFeatureParams(const std::string &path, const std::string &filename, wave::FeatureExtractorParams &params);

void loadBinnerParams(const std::string &path, const std::string &filename, wave::IcosahedronBinnerParams &params);

void setupFeatureParameters(wave::FeatureExtractorParams &param);

}

#endif //WAVE_CONFIG_UTILS_HPP
