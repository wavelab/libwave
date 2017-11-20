#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#include <vector>
#include <array>
#include <algorithm>
#include <utility>
#include <chrono>
#include <limits>
#include <iostream>
#include <fstream>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <type_traits>
#include <memory>
#include <set>
#include <exception>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>

#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/odometry/kdtreetype.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/laser_odom_residuals.hpp"
#include "wave/odometry/se3_residuals.hpp"
#include "wave/odometry/loss_functions.hpp"
#include "wave/odometry/kernels.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"
#include "wave/utils/math.hpp"

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <ceres/rotation.h>

namespace wave {

using IMUMeasurement = Measurement<Vec6, char>;
using unlong = unsigned long;

struct LaserOdomParams {
    // Optimizer parameters
    int opt_iters = 25;     // How many times to refind correspondences
    float diff_tol = 1e-6;  // norm of transform vector must change by more than this to continue
    float huber_delta = 0.2;
    float max_correspondence_dist = 0.4;  // correspondences greater than this are discarded
    double max_residual_val = 0.1;        // Residuals with an initial error greater than this are not used.
    double rotation_stiffness = 1;
    double translation_stiffness = 1;
    double T_z_multiplier = 1;
    double T_y_multiplier = 1;
    double RP_multiplier = 1;
    double decay_parameter = 1;
    int min_residuals = 30;
    double local_map_range = 10000;  // Maximum range of features to keep. square metres

    // Sensor parameters
    float scan_period = 0.1;  // Seconds
    int max_ticks = 36000;    // encoder ticks per revolution
    unlong n_ring = 32;       // number of laser-detector pairs

    // Feature extraction parameters
    float occlusion_tol = 0.1;    // Radians
    float occlusion_tol_2 = 0.1;  // m^2. Distance between points to initiate occlusion check
    float max_line_dist = 0.5;    // Radians. Points defining the lines must be at least this close
    float parallel_tol = 0.002;   // ditto
    float keypt_radius = 0.05;    // m2. Non maximum suppression distance
    float edge_tol = 0.1;         // Edge features must have score higher than this
    float flat_tol = 0.1;         // Plane features must have score lower than this
    float int_edge_tol = 2;       // Intensity edge features must have score greater than this
    float int_flat_tol = 0.1;     // Intensity edges must have range score lower than this
    int n_edge = 40;              // How many edge features to pick out per ring
    int n_flat = 100;             // How many plane features to pick out per ring
    int n_int_edge = 0;           // How many intensity edges to pick out per ring
    unlong knn = 5;               // 1/2 nearest neighbours for computing curvature
    unlong key_radius = 5;        // minimum number of points between keypoints on the same laser ring
    float map_density = 0.01;     // Minimum l2squared spacing of features kept for odometry
    // one degree. Beam spacing is 1.33deg, so this should be sufficient
    double azimuth_tol = 0.0174532925199433;    // Minimum azimuth difference across correspondences
    uint16_t TTL = 1;             // Maximum life of feature in local map with no correspondences
    double iso_var = 0.005;       // Variance to use if weighing is set.

    // Setting flags
    bool imposePrior = false;             // Whether to add a prior constraint on transform from the previous scan match
    bool visualize = false;               // Whether to run a visualization for debugging
    bool output_trajectory = false;       // Whether to output solutions for debugging/plotting
    bool output_correspondences = false;  // Whether to output correpondences for debugging/plotting
    bool only_extract_features = false;   // If set, no transforms are calculated
    bool use_weighting = false;
};

class LaserOdom {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LaserOdom(const LaserOdomParams params);
    ~LaserOdom();
    void addPoints(const std::vector<PointXYZIR> &pts, const int tick, TimeType stamp);
    void addIMU(std::vector<double> linacc, Quaternion orientation);
    std::vector<std::vector<std::vector<PointXYZIT>>> feature_points;  // edges, flats;
    std::vector<FeatureKDTree<double>> prv_feature_points;             // prv_edges, prv_flats;
    // The transform is T_start_end
    Transformation cur_transform, prev_transform;

    bool new_features = false;

    void rollover(TimeType stamp);
    bool match();
    void registerOutputFunction(std::function<void(const TimeType *const,
                                                   const Transformation *const,
                                                   const pcl::PointCloud<pcl::PointXYZI> *const)> output_function);
    void registerOutputFunction(std::function<void()> output_function);

    // Shared memory
    pcl::PointCloud<pcl::PointXYZI> undistorted_cld;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> undis_features;  // undis_edges, undis_flats;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> map_features;    // map_edges, map_flats;
    std::vector<std::vector<std::vector<double>>> output_corrs;
    TimeType undistorted_stamp;
    Transformation undistort_transform;
    double covar[144];  // use lift jacobians to reduce covariance coming out of ceres

    void updateParams(const LaserOdomParams);
    LaserOdomParams getParams();

    const uint32_t N_SIGNALS = 2;
    const uint32_t N_SCORES = 3;
    const uint32_t N_FEATURES = 5;

 private:
    // Visualizer elements, not allocated unless used
    PointCloudDisplay *display;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_viz, cur_viz;
    void updateViz();
    // Output trajectory file
    std::ofstream file;
    Eigen::IOFormat *CSVFormat;

    // Flow control
    std::atomic_bool continue_output;
    bool fresh_output = false;
    std::mutex output_mutex;
    std::condition_variable output_condition;
    std::unique_ptr<std::thread> output_thread;
    void spinOutput();
    std::function<void()> f_output;
    void undistort();

    // ceres optimizer stuff
    std::vector<std::pair<const double *, const double *>> covariance_blocks;

    LaserOdomParams param;
    bool initialized = false;
    int prv_tick = std::numeric_limits<int>::max();
    std::vector<double> scale_lookup;

    void computeScores();
    void prefilter();
    void generateFeatures();
    void buildTrees();
    bool findCorrespondingPoints(const Vec3 &query,
                                 const uint32_t &f_idx,
                                 std::vector<size_t> *index);

    PCLPointXYZIT applyIMU(const PCLPointXYZIT &pt);
    void transformToStart(const double *const pt, const uint16_t tick, double *output, const Vec6 &twist);
    void transformToEnd(const double *const pt, const uint16_t tick, double *output, const Vec6 &twist);

    // store for the IMU integral
    MeasurementContainer<IMUMeasurement> imu_trans;

    std::vector<double> lin_vel = {0, 0, 0};
    TimeType prv_time, cur_time;
    static float l2sqrd(const PCLPointXYZIT &p1, const PCLPointXYZIT &p2);
    static float l2sqrd(const PCLPointXYZIT &pt);
    static PCLPointXYZIT scale(const PCLPointXYZIT &pt, const float scale);
    void flagNearbyPoints(const unlong f_idx, const unlong ring, const unlong p_idx);

    // The input, in order of processing

    // Input set of points. Grouped by laser
    std::vector<pcl::PointCloud<PCLPointXYZIT>> cur_scan;
    // The range and intensity signals, grouped by kernel and ring
    std::vector<std::vector<std::vector<double>>> signals;
    // The resulting scores for each signal, grouped by kernel and ring
    std::vector<std::vector<std::vector<double>>> scores;
    // Whether points are still considered candidates. Required to avoid picking neighbouring points
    std::vector<std::vector<std::vector<bool>>> valid_pts;
    // Shared pointers to scoring kernels
    std::vector<std::shared_ptr<Eigen::Tensor<double, 1>>> kernels;
    // Container to sort scores with. Each is built depending on feature specification
    std::vector<std::vector<std::vector<std::pair<unlong, double>>>> filtered_scores;
    void buildFilteredScore(const std::vector<bool> &valid, const uint32_t &f_idx, const uint32_t &ring);

    std::vector<std::shared_ptr<kd_tree_t>> feature_idx;
    // This is container to hold indices of for each feature used in the optimization
    // It is indexed by feature_id, then by ring_id, then by correspondence.
    // It contains a vector of indices, the first is the feature point, the next are the indices of the
    // map or prv feature points it corresponds to
    std::vector<std::vector<std::vector<std::vector<uint64_t>>>> feature_corrs;
    enum AssociationStatus { CORRESPONDED, UNCORRESPONDED };

    // Eventually connect to yamls to be able to change
    // kernels & policies very quickly
    enum SelectionPolicy { HIGH_POS, HIGH_NEG, NEAR_ZERO };
    enum Kernel { LOAM, LOG, FOG };
    enum ResidualType { PointToLine, PointToPlane };
    struct Criteria {
        Kernel kernel;
        SelectionPolicy sel_pol;
        double threshold;
    };

    struct FeatureDefinition {
        // score criteria. First item defines sort, any other criteria
        // is checked for validity.
        std::vector<Criteria> criteria;
        // Type of residual to use with this feature type
        ResidualType residual;
        int n_limit;
    };
    std::vector<FeatureDefinition> feature_definitions;

    std::vector<std::vector<std::pair<uint64_t, AssociationStatus>>> feature_association;
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
