#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#ifndef EIGEN_USE_THREADS
#define EIGEN_USE_THREADS
#endif

#include <unsupported/Eigen/CXX11/Tensor>
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
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>

#include "wave/containers/measurement_container.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/odometry/kdtreetype.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/kernels.hpp"
#include "wave/odometry/integrals.hpp"
#include "wave/odometry/sensor_model.hpp"
#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp.hpp"
#include "wave/optimization/ceres/odom_gp_twist/point_to_plane_gp.hpp"
#include "wave/optimization/ceres/odom_gp_twist/constant_velocity.hpp"
#include "wave/optimization/ceres/loss_function/bisquare_loss.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/data.hpp"

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <ceres/rotation.h>
#include <wave/kinematics/constant_velocity_gp_prior.hpp>

namespace wave {

using unlong = unsigned long;
using TimeType = std::chrono::steady_clock::time_point;

struct LaserOdomParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// The covariance matrix for noise on velocity
    Mat6 Qc = Mat6::Identity();
    /// inverse stored to save repeated steps
    Mat6 inv_Qc = Mat6::Identity();
    // Optimizer parameters
    // How many states per revolution to optimize over
    // There must be at minimum two.
    uint32_t num_trajectory_states = 3;
    // How many scans to optimize over, must be at least 1, obviously
    uint32_t n_window = 1;

    int opt_iters = 25;         // How many times to refind correspondences
    int max_inner_iters = 100;  // How many iterations of ceres to run with each set of correspondences
    float diff_tol = 1e-4;      // If the transform from one iteration to the next changes less than this,
    // skip the next iterations
    int solver_threads = 0;               // How many threads ceres should use. 0 or less results in using the
                                          // number of logical threads on the machine
    float robust_param = 0.2;             // Hyper-parameter for bisquare (Tukey) loss function
    float max_correspondence_dist = 0.4;  // correspondences greater than this are discarded
    double max_residual_val = 0.1;        // Residuals with an initial error greater than this are not used.
    int min_residuals = 30;               // Problem will be reset if the number of residuals is less than this
    double local_map_range = 10000;       // Maximum range of features to keep. square metres

    // Sensor parameters
    float scan_period = 0.1;  // Seconds
    int max_ticks = 36000;    // encoder ticks per revolution
    unlong n_ring = 32;       // number of laser-detector pairs

    RangeSensorParams sensor_params;

    // Feature extraction parameters
    unlong variance_window = 11;
    bool limit_rng_var = false;
    bool limit_int_var = false;
    double variance_limit_rng = 1;
    double variance_limit_int = 1;

    unlong angular_bins = 12;
    float min_intensity = 10.0;
    float max_intensity = 50.0;
    float occlusion_tol = 0.1;      // Radians
    float occlusion_tol_2 = 1;      // m. Distance between points to initiate occlusion check
    float parallel_tol = 0.002;     // ditto
    double edge_tol = 0.1;          // Edge features must have score higher than this
    double flat_tol = 0.1;          // Plane features must have score lower than this
    double int_edge_tol = 2;        // Intensity edge features must have score greater than this
    double int_flat_tol = 0.1;      // Intensity edges must have range score lower than this
    int n_edge = 40;                // How many edge features to pick out per ring
    int n_flat = 100;               // How many plane features to pick out per ring
    int n_int_edge = 0;             // How many intensity edges to pick out per ring
    unlong knn = 5;                 // 1/2 nearest neighbours for computing curvature
    unlong key_radius = 5;          // minimum number of points between keypoints on the same laser ring
    float edge_map_density = 0.01;  // Minimum l2squared spacing of features kept for odometry
    float flat_map_density = 0.25;
    // one degree. Beam spacing is 1.33deg, so this should be sufficient
    double azimuth_tol = 0.0174532925199433;  // Minimum azimuth difference across correspondences
    uint16_t TTL = 1;                         // Maximum life of feature in local map with no correspondences

    double min_eigen = 100.0;
    double max_extrapolation = 0.0;  // Increasing this number from 0 will allow a bit of extrapolation

    // Setting flags
    bool visualize = false;               // Whether to run a visualization for debugging
    bool output_trajectory = false;       // Whether to output solutions for debugging/plotting
    bool output_correspondences = false;  // Whether to output correpondences for debugging/plotting
    bool only_extract_features = false;   // If set, no transforms are calculated
    bool use_weighting = false;           // If set, pre-whiten residuals
    bool lock_first = true;               // If set, assume starting position is identity
    bool plot_stuff = false;              // If set, plot things for debugging
    bool solution_remapping = false;      // If set, use solution remapping
    bool motion_prior = true;             // If set, use a constant velocity prior
    bool no_extrapolation = false;  // If set, discard any point match whose correspondences do not bound the point
                                    /**
                                     * If set instead of matching edge points to lines, match edge points to a plane
                                     * defined by the original line points and the origin
                                     */
    bool treat_lines_as_planes = false;
};

class LaserOdom {
    // true uses transform class with some approximations for exp map, etc
    // false uses full analytical expressions wherever possible
    using T_TYPE = Transformation<Eigen::Matrix<double, 3, 4>, true>;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit LaserOdom(const LaserOdomParams params);
    ~LaserOdom();
    void addPoints(const std::vector<PointXYZIR> &pts, int tick, TimeType stamp);

    std::vector<std::vector<std::vector<PointXYZIT>>> feature_points;  // edges, flats;

    std::vector<FeatureKDTree<double>> prv_feature_points;  // previous features now in map

    struct Trajectory {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        T_TYPE pose;
        Vec6 vel;
    };

    struct TrajDifference {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vec12 hat_multiplier, candle_multiplier;
    };

    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> cur_trajectory;
    std::vector<TrajDifference, Eigen::aligned_allocator<TrajDifference>> cur_difference;
    Vec6 current_twist;

    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> prev_trajectory;

    T_TYPE inv_prior_pose;
    Vec6 prior_twist;

    void rollover(TimeType stamp);
    bool match();
    void registerOutputFunction(std::function<void()> output_function);

    // Shared memory
    std::mutex output_mutex;
    pcl::PointCloud<pcl::PointXYZI> undistorted_cld;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>>>
      undis_features;  // undis_edges, undis_flats;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>>>
      map_features;  // map_edges, map_flats;
    std::vector<std::vector<std::vector<double>>> output_corrs;
    std::vector<double> output_eigen;
    TimeType undistorted_stamp;
    Transformation<> undistort_transform;
    Vec6 undistort_velocity;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covar;

    void updateParams(const LaserOdomParams);
    LaserOdomParams getParams();

    const uint32_t N_SIGNALS = 2;
    const uint32_t N_SCORES = 5;
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
    std::condition_variable output_condition;
    std::unique_ptr<std::thread> output_thread;
    void spinOutput();
    std::function<void()> f_output;
    void undistort();

    // ceres optimizer stuff
    std::vector<Vec12, Eigen::aligned_allocator<Vec12>> param_blocks;
    // Preallocated memory for the residuals
    std::vector<wave_optimization::SE3PointToLineGPObjects,
                Eigen::aligned_allocator<wave_optimization::SE3PointToLineGPObjects>>
      PtLMem;
    std::vector<wave_optimization::SE3PointToPlaneGPObjects,
                Eigen::aligned_allocator<wave_optimization::SE3PointToPlaneGPObjects>>
      PtPMem;

    LaserOdomParams param;
    bool initialized = false, full_revolution = false;
    int prv_tick = std::numeric_limits<int>::max();

    void computeScores();
    void prefilter();
    void generateFeatures();
    void buildTrees();
    bool findCorrespondingPoints(const Vec3 &query, const uint32_t &f_idx, std::vector<size_t> *index);
    bool outOfBounds(const Vec3 &query, const uint32_t &f_idx, const std::vector<size_t> &index);

    PCLPointXYZIT applyIMU(const PCLPointXYZIT &pt);
    void transformToMap(
      const double *const pt, const uint32_t tick, double *output, uint32_t &k, uint32_t &kp1, double &tau);
    void transformToMap(const double *const pt, const uint32_t tick, double *output);
    void transformToCurLidar(const double *const pt, const uint32_t tick, double *output);
    void resetTrajectory();
    void copyTrajectory();
    void applyRemap();
    // Do some calculations for transforms ahead of time
    void updateDifferences();
    void updateOperatingPoint();

    // Lidar Sensor Model
    std::shared_ptr<RangeSensor> range_sensor;
    // Motion Model
    std::vector<wave_kinematics::ConstantVelocityPrior,
                Eigen::aligned_allocator<wave_kinematics::ConstantVelocityPrior>>
      cv_vector;
    std::vector<double> trajectory_stamps;

    Mat6 sqrtinfo;
    TimeType prv_time, cur_time;

    void getTransformIndices(const uint32_t &tick, uint32_t &start, uint32_t &end, double &frac);

    static float l2sqrd(const PCLPointXYZIT &p1, const PCLPointXYZIT &p2);

    void flagNearbyPoints(const unlong f_idx, const unlong ring, const unlong p_idx);

    // The input, in order of processing

    // Input scan as an eigen tensor with dimensions (channels, rings, max_azimuth)
    std::vector<uint32_t> counters;
    Eigen::Tensor<float, 3> cur_scan;

    // dimensions (channels, rings, max_azimuth)
    Eigen::Tensor<float, 3> signals;

    // The resulting scores for each signal, grouped by kernel and ring
    Eigen::Tensor<float, 3> scores;

    // Whether points are still considered candidates. Required to avoid picking neighbouring points
    Eigen::Tensor<bool, 2> valid_pts;

    // Scoring kernels, indexed along 1st dimension
    Eigen::Tensor<float, 2> kernels;

    // Container to sort scores with. Each is built depending on feature specification
    std::vector<std::vector<std::vector<std::pair<unlong, double>>>> filtered_scores;

    // This is container to hold indices of for each feature used in the optimization
    // It is indexed by feature_id, then by ring_id, then by correspondence.
    // It contains a vector of indices, the first is the feature point, the next are the indices of the
    // map or prv feature points it corresponds to
    std::vector<std::vector<std::vector<std::vector<uint64_t>>>> feature_corrs;
    enum AssociationStatus { CORRESPONDED, UNCORRESPONDED };
    std::vector<std::vector<std::pair<uint64_t, AssociationStatus>>> feature_association;

    void buildFilteredScore(const uint32_t &f_idx);
    std::vector<std::shared_ptr<kd_tree_t>> feature_idx;

    // Eventually connect to yamls to be able to change
    // kernels & policies very quickly
    enum SelectionPolicy { HIGH_POS, HIGH_NEG, NEAR_ZERO };
    // todo: Rename Kernel, not just a kernel anymore
    enum Kernel { LOAM, LOG, FOG, RNG_VAR, INT_VAR };
    enum ResidualType { PointToLine, PointToPlane };
    struct Criteria {
        Kernel kernel;
        SelectionPolicy sel_pol;
        double *threshold;
    };

    struct FeatureDefinition {
        // score criteria. First item defines sort, any other criteria
        // is checked for validity.
        std::vector<Criteria> criteria;
        // Type of residual to use with this feature type
        ResidualType residual;
        int *n_limit;
    };
    std::vector<FeatureDefinition> feature_definitions;
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
