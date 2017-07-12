#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#include <vector>
#include <array>
#include <algorithm>
#include <utility>
#include <chrono>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "wave/matching/pointcloud_display.hpp"
#include "wave/odometry/kdtreetype.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/laser_odom_residuals.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/utils/math.hpp"

namespace wave {

using IMUMeasurement = Measurement<Vec6, char>;
using unlong = unsigned long;

struct LaserOdomParams {
    // Optimizer parameters
    int opt_iters = 25;          // How many times to refind correspondences
    float diff_tol = 1e-6;       // norm of transform vector must change by more than this to continue
    float huber_delta = 0.1;
    float max_correspondence_dist = 0.2;  // correspondences greater than this are discarded

    // Sensor parameters
    float scan_period = 0.1;     // Seconds
    int max_ticks = 35999;  // encoder ticks per revolution
    unlong n_ring = 32;    // number of laser-detector pairs

    // Feature extraction parameters
    float occlusion_tol = 0.1;   // Don't know units
    float parallel_tol = 0.002;  // ditto
    float keypt_radius = 0.05;   // m2
    float edge_tol = 0.1;  // Edge features must have score higher than this
    float flat_tol = 0.1;  // Plane features must have score lower than this
    int n_edge = 40;       // How many edge features to pick out per ring
    int n_flat = 100;      // How many plane features to pick out per ring
    unlong knn = 5;        // 1/2 nearest neighbours for computing curvature
    bool visualize = false;   //Whether to run a visualization for debugging
};

class LaserOdom {
 public:
    LaserOdom(const LaserOdomParams params);
    void addPoints(const std::vector<PointXYZIR> &pts,
                   const int tick,
                   TimeType stamp);
    void addIMU(std::vector<double> linacc, Quaternion orientation);
    std::vector<PointXYZIT> edges, flats;
    FeatureKDTree<double> prv_edges, prv_flats;
    // transform is stored as an axis-angle rotation [012] and a
    // displacement [345]
    // The transform is T_start_end
    std::array<double, 6> cur_transform;
    bool new_features = false;

 private:
    // Visualizer elements, not allocated unless used
    PointCloudDisplay* display;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_viz, cur_viz;
    void updateViz();

    LaserOdomParams param;
    bool initialized = false;
    int prv_tick = std::numeric_limits<int>::max();

    void transformToStart();

    void rollover(TimeType stamp);
    void resetIMU(TimeType stamp);
    void computeCurvature();
    void prefilter();
    void generateFeatures();
    void buildTrees();
    bool match();

    PCLPointXYZIT applyIMU(const PCLPointXYZIT &pt);

    // store for the IMU integral
    MeasurementContainer<IMUMeasurement> imu_trans;

    std::vector<double> lin_vel = {0, 0, 0};
    TimeType prv_time, cur_time;
    static float l2sqrd(const PCLPointXYZIT &p1, const PCLPointXYZIT &p2);
    static float l2sqrd(const PCLPointXYZIT &pt);
    static PCLPointXYZIT scale(const PCLPointXYZIT &pt, const float scale);
    void flagNearbyPoints(const unlong ring, const unlong index);
    std::vector<std::vector<std::pair<bool, float>>> cur_curve;
    std::vector<std::vector<std::pair<unlong, float>>> filter;
    std::vector<pcl::PointCloud<PCLPointXYZIT>> cur_scan;
    kd_tree_t *edge_idx;
    kd_tree_t *flat_idx;
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
