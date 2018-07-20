#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#ifndef EIGEN_USE_THREADS
#define EIGEN_USE_THREADS
#endif

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
#include <unsupported/Eigen/CXX11/Tensor>

#include <nabo/nabo.h>

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <ceres/rotation.h>

#include "wave/geometry/transformation.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/sensor_model.hpp"
#include "wave/odometry/transformer.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/implicit_geometry/implicit_plane.hpp"
#include "wave/odometry/implicit_geometry/implicit_line.hpp"
#include "wave/optimization/ceres/odom_gp_twist/constant_velocity.hpp"
#include "wave/optimization/ceres/local_params/null_SE3_parameterization.hpp"
#include "wave/optimization/ceres/local_params/line_parameterization.hpp"
#include "wave/optimization/ceres/local_params/plane_parameterization.hpp"
#include "wave/optimization/ceres/loss_function/bisquare_loss.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

using unlong = unsigned long;

struct LaserOdomParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// The covariance matrix for noise on velocity
    Mat6 Qc = Mat6::Identity();
    /// inverse stored to save repeated steps
    Mat6 inv_Qc = Mat6::Identity();
    // Optimizer parameters
    // How many states per revolution to optimize over
    // There must be at minimum two (start and end. Consecutive scans share a state for end-start boundary)
    uint32_t num_trajectory_states = 3;
    // How many scans to optimize over, must be at least 2 (scan-to-scan matching)
    uint32_t n_window = 2;

    int opt_iters = 25;         // How many times to refind correspondences
    int max_inner_iters = 100;  // How many iterations of ceres to run with each set of correspondences
    float diff_tol = 1e-5;      // If the transform from one iteration to the next changes less than this,
    // skip the next iterations
    int solver_threads = 0;               // How many threads ceres should use. 0 or less results in using the
                                          // number of logical threads on the machine
    int min_features = 300;               // How many features are required
    float robust_param = 0.2;             // Hyper-parameter for bisquare (Tukey) loss function
    float max_correspondence_dist = 1;  // correspondences greater than this are discarded
    double max_residual_val = 0.1;        // Residuals with an initial error greater than this are not used.
    int min_residuals = 30;               // Problem will be reset if the number of residuals is less than this

    // Sensor parameters
    float scan_period = 0.1;  // Seconds
    int max_ticks = 36000;    // encoder ticks per revolution
    unlong n_ring = 32;       // number of laser-detector pairs

    RangeSensorParams sensor_params;

    // one degree. Beam spacing is 1.33deg, so this should be sufficient
    double azimuth_tol = 0.0174532925199433;  // Minimum azimuth difference across correspondences
    uint16_t TTL = 1;                         // Maximum life of feature in local map with no correspondences

    double min_eigen = 100.0;
    double max_extrapolation = 0.0;  // Increasing this number from 0 will allow a bit of extrapolation

    // Setting flags
    bool output_trajectory = false;       // Whether to output solutions for debugging/plotting
    bool output_correspondences = false;  // Whether to output correpondences for debugging/plotting
    bool only_extract_features = false;   // If set, no transforms are calculated
    bool use_weighting = false;           // If set, pre-whiten residuals
    bool plot_stuff = false;              // If set, plot things for debugging
    bool motion_prior = true;             // If set, use a constant velocity prior
    bool no_extrapolation = false;  // If set, discard any point match whose correspondences do not bound the point
                                    /**
                                     * If set instead of matching edge points to lines, match edge points to a plane
                                     * defined by the original line points and the origin
                                     */
    bool treat_lines_as_planes = false;
};

class LaserOdom {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit LaserOdom(const LaserOdomParams params, const FeatureExtractorParams feat_params,
                           const TransformerParams transformer_params);
    ~LaserOdom();
    void addPoints(const std::vector<PointXYZIR> &pts, int tick, TimeType stamp);
    void rollover(TimeType stamp);
    void registerOutputFunction(std::function<void()> output_function);
    void updateParams(const LaserOdomParams);
    LaserOdomParams getParams();

    VecE<PoseVel> cur_trajectory, prev_trajectory;

    Vec6 prior_twist;

    // Shared memory
    std::mutex output_mutex;
    pcl::PointCloud<pcl::PointXYZI> undistorted_cld;
    VecE<pcl::PointCloud<pcl::PointXYZ>> undis_features;
    VecE<Vec6f> geometry_landmarks;

    PoseVelStamped undistort_state;

    const uint32_t N_SIGNALS = 2;
    const uint32_t N_FEATURES = 5;
    const uint32_t MAX_POINTS = 2200;

 private:
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

    LaserOdomParams param;
    bool initialized = false;
    int prv_tick = std::numeric_limits<int>::max();

    FeatureExtractor feature_extractor;
    Transformer transformer;

    void updateFeatureCandidates();
    void prepTrajectory(const TimeType &stamp);
    bool match(const TimeType &stamp);
    void buildResiduals(ceres::Problem &problem);
    //todo think about how to reuse residuals
    Vec<std::shared_ptr<ceres::CostFunction>> costs;
    std::vector<std::shared_ptr<ceres::LocalParameterization>> local_params;
    std::vector<std::shared_ptr<ceres::LossFunction>> loss_functions;
    bool runOptimization(ceres::Problem &problem);

    void extendFeatureTracks(const Eigen::MatrixXi &indices, const Eigen::MatrixXf &dist, uint32_t feat_id);
    void createNewFeatureTracks(const Eigen::MatrixXi &indices, const Eigen::MatrixXf &dist, uint32_t feat_id);
    void mergeFeatureTracks(uint32_t feat_id);

    void undistort();

    void resetTrajectory();
    void copyTrajectory();

    // Lidar Sensor Model
    std::shared_ptr<RangeSensor> range_sensor;
    // Motion Model
    VecE<wave_kinematics::ConstantVelocityPrior> cv_vector;
    Vec<float> trajectory_stamps;

    Mat6 sqrtinfo;
    Vec<TimeType> scan_stamps_chrono;
    Vec<float> scan_stampsf;

    // Input scan as an vector of eigen tensors with dimensions rings x (channels, max points in ring)
    Vec<int> counters;
    VecE<Eigen::Tensor<float, 2>> cur_scan;

    // rings x (channels, points in ring)
    VecE<Eigen::Tensor<float, 2>> signals;

    // indices of points to use for features, indexed by feature type and ring
    Vec<VecE<Eigen::Tensor<int, 1>>> indices;


    /// This stores candidate feature points before they are put into feat_pts container.
    VecE<Eigen::Tensor<float, 2>> cur_feature_candidates, prev_feature_candidates;
    VecE<Eigen::Tensor<float, 2>> cur_feature_candidatesT, prev_feature_candidatesT;
    Vec<std::shared_ptr<Eigen::Map<Eigen::MatrixXf>>> cur_feat_map, prev_feat_map;
    Vec<Vec<Eigen::Map<MatXf>>> feat_T_map;
    /// stores the index of the feature track associated with each feature point. -1 if not associated with a feature track
    Vec<Vec<int>> cur_feat_idx, prev_feat_idx;
    /**
     * feat_pts and feat_pts_T are sets of indexed tensors, first by scan then feature type
     */
    Vec<VecE<Eigen::Tensor<float, 2>>> feat_pts, feat_pts_T;
    /// interp_factors store interpolation factors for use during optimization
    Vec<VecE<Eigen::Tensor<float, 2>>> interp_factors;

    Vec<Nabo::NNSearchF*> cur_kd_idx, curm1_kd_idx, ave_kd_idx;

    Vec<ResidualType> feature_residuals;

    //storage for average feature points. 3 x N
    VecE<MatXf> ave_pts;
    Vec<Vec<uint32_t>> track_ids;
    // indexed by feature type and then track_id
    Vec<VecE<FeatureTrack>> feature_tracks;
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
