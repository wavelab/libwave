#ifndef WAVE_LASERODOM_HPP
#define WAVE_LASERODOM_HPP

#ifndef EIGEN_USE_THREADS
#define EIGEN_USE_THREADS
#endif

#include <vector>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <type_traits>
#include <utility>

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
#include "wave/odometry/icosahedron_binner.hpp"
#include "wave/odometry/PointXYZIR.hpp"
#include "wave/odometry/PointXYZIT.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/odometry/sensor_model.hpp"
#include "wave/odometry/transformer.hpp"
#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/geometry/plane.hpp"
#include "wave/odometry/geometry/line.hpp"
#include "wave/odometry/odometry_callback.hpp"
#include "wave/optimization/ceres/odom_gp/constant_velocity.hpp"
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

    /// Information matrix for weighing first twist.
    Mat6 prior_info = Mat6::Identity();
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
    double max_planar_residual_val = 0.1;        // Residuals with an initial error greater than this are not used.
    double max_linear_residual_val = 0.1;        // Residuals with an initial error greater than this are not used.
    int min_residuals = 30;               // Problem will be reset if the number of residuals is less than this

    // Sensor parameters
    //todo get rid of this (should be part of the feature extractor params, not here)
    unlong n_ring = 32;       // number of laser-detector pairs

    RangeSensorParams sensor_params;

    // one degree. Beam spacing is 1.33deg, so this should be sufficient
    double elevation_tol = 0.0174532925199433;  // Minimum azimuth difference across correspondences
    uint16_t TTL = 1;                         // Maximum life of feature in local map with no correspondences

    /// parameters for feature track merging
    float max_planar_dist_threshold = 0.2f;
    float max_planar_ang_threshold = 0.03f;

    float max_linear_dist_threshold = 0.1;
    float max_linear_ang_threshold = 0.025;
    float ang_scaling_param = 10;

    IcosahedronBinnerParams binner_params;

    uint32_t min_new_points = 5;

    // Setting flags
    bool only_extract_features = false;   // If set, no transforms are calculated
    bool use_weighting = false;           // If set, pre-whiten residuals
    bool print_opt_sum = false;              // If set, plot things for debugging
};

class LaserOdom {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit LaserOdom(const LaserOdomParams params, const FeatureExtractorParams feat_params,
                           const TransformerParams transformer_params);
    ~LaserOdom();
    void addPoints(const std::vector<PointXYZIR> &pts, int tick, TimeType stamp);
    void updateTracks();
    void rollover(TimeType stamp);
    void registerOutputFunction(std::function<void()> output_function);
    void updateParams(const LaserOdomParams);
    LaserOdomParams getParams() const;

    VecE<PoseVel> cur_trajectory, prev_trajectory;

    Vec6 prior_twist;
    Mat6 twist_covar;
    double prev_delta_t;

    // Shared memory
    std::mutex output_mutex;
    pcl::PointCloud<pcl::PointXYZI> undistorted_cld;
    VecE<pcl::PointCloud<pcl::PointXYZ>> undis_features;
    Vec<VecE<FeatureTrack>> undis_tracks;
    VecE<PoseVelStamped> undistort_trajectory;
    VecE<pcl::PointCloud<pcl::PointXYZ>> undis_candidates_cur, undis_candidates_prev;

    const uint32_t N_SIGNALS = 2;
    const uint32_t N_FEATURES = 5;
    const uint32_t MAX_POINTS = 2200;

 private:
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
    void trackResiduals(ceres::Problem &problem, uint32_t f_idx, VecE <FeatureTrack> &track_list);
    void buildResiduals(ceres::Problem &problem);
    //todo think about how to reuse residuals
    Vec<std::shared_ptr<ceres::CostFunction>> costs;
    std::vector<std::shared_ptr<ceres::LocalParameterization>> local_params;
    std::vector<std::shared_ptr<ceres::LossFunction>> loss_functions;
    bool runOptimization(ceres::Problem &problem, ceres::Solver::Summary &summary);
    void calculateCovariance(ceres::Problem &problem);

    template<typename Derived, typename Derived1, typename Derived2>
    bool findLineCorrespondences(std::vector<uint32_t> &matches, std::vector<int> &used_points,
                                     const Eigen::MatrixBase<Derived> &index,
                                     const Eigen::MatrixBase<Derived1> &dist,
                                     const Eigen::MatrixBase<Derived2> &points);

    template<typename Derived, typename Derived1, typename Derived2>
    bool findPlaneCorrespondences(std::vector<uint32_t> &matches, Vec<int> &used_points,
                                      const Eigen::MatrixBase<Derived> &index,
                                      const Eigen::MatrixBase<Derived1> &dist,
                                      const Eigen::MatrixBase<Derived2> &points);

    void extendFeatureTracks(const Eigen::MatrixXi &indices, const MatXf &distances, uint32_t feat_id,
                                 const bool large_tol);

    void createNewGeometry(const MatXf &points, const Vec<uint32_t> &indices, ResidualType residual_type, Vec6 &geometry);

    void createNewFeatureTracks(const Eigen::MatrixXi &indices, const MatXf &distances, uint32_t feat_id,
                                    const Nabo::NNSearchF *cur_knn_tree, const bool large_tol);
    /**
     * Removes any correspondences to the current scan
     */
    void clearVolatileTracks();

    void calculateLineSimilarity(const Vec6 &geo1, const Vec6 &geo2, float &dist_cost, float &dir_cost);
    void calculatePlaneSimilarity(const Vec6 &geo1, const Vec6 &geo2, float &dist_cost, float &dir_cost);

    void mergeFeatureTracks(VecE <FeatureTrack> &tracks, uint32_t feat_id);

    void setupSkipPoint(uint32_t feat_id);

    void undistort();

    void resetTrajectory();

    template<class S_TYPE, class D_TYPE>
    void copyTrajectory(const VecE<S_TYPE> &src, VecE<D_TYPE> &dst);

    // Lidar Sensor Model
    std::shared_ptr<RangeSensor> range_sensor;
    // Motion Model
    std::shared_ptr<wave_kinematics::ConstantVelocityPrior> cv_model;
    Vec<float> trajectory_stamps;

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
    VecE<MatXf> cur_feature_candidatesT, prev_feature_candidatesT;
    /// stores the index of the feature track associated with each candidate feature point. -1 if not associated with a feature track
    Vec<Vec<int>> cur_feat_idx, prev_feat_idx;
    /**
     * feat_pts and feat_pts_T are sets of indexed tensors, first by scan then feature type
     */

    Vec<VecE<Eigen::Tensor<float, 2>>> feat_pts;
    Vec<VecE<MatXf>> feat_pts_T;
    Vec<Vec<Vec<bool>>> skip_point;

    Vec<Vec<VecE<MatX>>> ptT_jacobians;
    Vec<Vec<float>> jacobian_stamps;

    Vec<ResidualType> feature_residuals;

    //storage for average feature points. 3 x N
    VecE<MatXf> ave_pts;
    // indexed by feature type and then track_id

    Vec<long> cm1_feat_pts_size;

    Vec<VecE<FeatureTrack>> feature_tracks;

    Vec<IcosahedronBinner> binner;

    void checkTrackValidity();
};

}  // namespace wave

#endif  // WAVE_LASERODOM_HPP
