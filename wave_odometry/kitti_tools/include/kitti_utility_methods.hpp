#ifndef WAVE_KITTI_UTILITY_METHODS_HPP
#define WAVE_KITTI_UTILITY_METHODS_HPP

#include <matplotlibcpp.h>

#include "wave/geography/world_frame_conversions.hpp"
#include "wave/odometry/laser_odom.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/config.hpp"

namespace wave {

struct TrackLengths {
    Vec<Vec<int>> lengths;
};

void updateVisualizer(const wave::LaserOdom *odom,
                      wave::PointCloudDisplay *display,
                      wave::VecE<wave::PoseVelStamped> *odom_trajectory,
                      Vec<TrackLengths> *track_lengths = nullptr,
                      VecE<PoseVel> *prev_odom_trajectory = nullptr);

void plotResults(const wave::VecE<wave::PoseVelStamped> &ground_truth,
                 const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotResults(const wave::VecE<wave::PoseStamped> &ground_truth,
                 const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotResults(const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotError(const wave::VecE<wave::PoseStamped> &ground_truth, const wave::VecE<wave::PoseVelStamped> &odom_trajectory);

void plotTrackLengths(const Vec<TrackLengths> &track_lengths);

wave::TimeType parseTime(const std::string &date_time);

wave::Mat34 poseFromGPS(const std::vector<double> &vals, const wave::Mat34 &reference, wave::Mat3 &R_ENU_CAR);

void fillGroundTruth(wave::VecE<wave::PoseVelStamped> &trajectory, const std::string &data_path);

}

#endif //WAVE_KITTI_UTILITY_METHODS_HPP
