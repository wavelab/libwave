#ifndef WAVE_KITTI_UTILITY_METHODS_HPP
#define WAVE_KITTI_UTILITY_METHODS_HPP

#include <matplotlibcpp.h>

#include "wave/geography/world_frame_conversions.hpp"
#include "wave/odometry/laser_odom.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/config.hpp"

namespace wave {

void LoadSensorParameters(const std::string &path, const std::string &filename, wave::RangeSensorParams &senparams);

void LoadParameters(const std::string &path, const std::string &filename, wave::LaserOdomParams &params);

void loadFeatureParams(const std::string &path, const std::string &filename, wave::FeatureExtractorParams &params);

void setupFeatureParameters(wave::FeatureExtractorParams &param);

void updateVisualizer(const wave::LaserOdom *odom,
                      wave::PointCloudDisplay *display,
                      wave::VecE<wave::PoseVelStamped> *odom_trajectory);

void plotResults(const wave::VecE<wave::PoseVelStamped> &ground_truth,
                 const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotResults(const wave::VecE<wave::PoseStamped> &ground_truth,
                 const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotResults(const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

wave::TimeType parseTime(const std::string &date_time);

wave::Mat34 poseFromGPS(const std::vector<double> &vals, const wave::Mat34 &reference);

void fillGroundTruth(wave::VecE<wave::PoseVelStamped> &trajectory, const std::string &data_path);

}

#endif //WAVE_KITTI_UTILITY_METHODS_HPP
