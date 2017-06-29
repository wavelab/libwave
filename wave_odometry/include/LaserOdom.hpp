#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#include <vector>
#include <algorithm>
#include <utility>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "PointXYZIR.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/utils/math.hpp"

namespace wave {

using IMUMeasurement = Measurement<Vec6, char>;
using unlong = unsigned long;
using PointType = pcl::PointXYZI;

struct LaserOdomParams {
    const int opt_iters = 25;
    const float scan_period = 0.1;     // Seconds
    const float occlusion_tol = 0.1;   //Don't know units
    const float parallel_tol = 0.002;  //ditto
    const float keypt_radius = 0.05;  //m2
    const float edge_tol = 0.1;  // Edge features must have score higher than this
    const float flat_tol = 0.1;  // Plane features must have score lower than this
    const int max_ticks = 3600;  // encoder ticks per revolution
    const int n_edge = 40;  // How many edge features to pick out
    const int n_flat = 100;  // How many plane features to pick out
    const unlong knn = 5;  // 1/2 nearest neighbours for computing curvature
    const unlong n_ring = 32; // number of laser-detector pairs
};

class LaserOdom {
 public:
    LaserOdom(const LaserOdomParams params);
    void addPoints(std::vector<PointXYZIR> pts,
                   const int tick,
                   const TimeType stamp);
    void addIMU(std::vector<double> linacc, Quaternion orientation);

 private:
    LaserOdomParams param;
    bool initialized = false;

    void transformToStart();

    void undistort();
    void rollover(const TimeType stamp);
    void resetIMU(const TimeType stamp);
    void computeCurvature();
    void prefilter();
    void generateFeatures();
    void match();

    PointType applyIMU(const PointType& pt, int tick);

    // store for the IMU integral
    MeasurementContainer<IMUMeasurement> imu_trans;

    std::vector<double> lin_vel = {0, 0, 0};
    TimeType prv_time, cur_time;
    static float l2sqrd(const PointType& p1, const PointType& p2);
    static float l2sqrd(const PointType& pt);
    static PointType scale(const PointType& pt, const float scale);
    void flagNearbyPoints(const unlong ring, const unlong index);
    std::vector<std::vector<std::pair<bool, float>>> cur_curve;
    std::vector<std::vector<std::pair<unlong, float>>> filter;
    std::vector<pcl::PointCloud<PointType>> cur_scan;
    pcl::PointCloud<PointType> edges, flats, prv_edges, prv_flats;
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
