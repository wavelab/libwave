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
    int opt_iters = 25;
    float scan_period = 0.1;     // Seconds
    float occlusion_tol = 0.1;  //Don't know units
    float parallel_tol = 0.002;
    const int max_ticks = 3600;  // encoder ticks per revolution
    unsigned int knn = 5;  // 1/2 nearest neighbours for computing curvature
    int n_ring = 32; // number of laser-detector pairs
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
    std::vector<std::vector<std::pair<bool, float>>> cur_curve;
    std::vector<std::vector<std::pair<unlong, float>>> filter;
    std::vector<pcl::PointCloud<PointType>> prv_scan, cur_scan;
};

}  // namespace wave

/*
 * If I understand this correctly this is the flow of data.
 *
 * Continously gather IMU measurements and put them into a measurement container
 *
 * Wait for some points that have an encoder tick of 0. Record timestamp as
 * start of
 * scan. For each new set of points that comes in, transform them by the imu
 * integration
 * amount and put in scan.
 *
 * Should only have keep a running integration of the IMU:
 * When a new revolution is started, zero out the integrator.
 * Then, for each new set of points, integrate the initial velocity + imu
 * accelerations
 * into a transform, and use to transform point "back" to start of scan.
 *
 * If a revolution is complete, perform optimization to arrive at constant
 * velocity
 * transform. Optimization is done with nearest neighbour over each tracker.
 * Could also optimize over IMU biases. Once done, identify any outliers from
 * any
 * tracker group and fuse
 *
 * Use a non-rigid transformation algorithm on the outliers (reformulating for
 * planes/lines
 * of course). Then segment by velocity and assign trackers ids, checking for
 * similarity to
 * existing trackers and fusing if some condition is met.
 *
 * Optionally, use ground tracker features to match against a map and produce an
 * absolute
 * constraint.
 */

#endif  // WAVE_LASERODOM_HPP
