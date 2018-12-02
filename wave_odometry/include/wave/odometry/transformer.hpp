/***
 * Purpose of this class is to abstract away transforming feature points ahead of querying for correspondences
 * and residual construction given the current operating point (trajectory). Depending on how fast the trajectory
 * is changing, this class will select a number of sample points to do the full, expensive GP transform interpolation,
 * then use linear interpolation between those sample points for a speedup.
 *
 * There are two conditions important to be considered quickly-changing; acceleration and angular rate
 */
#ifndef WAVE_TRANSFORMER_HPP
#define WAVE_TRANSFORMER_HPP

#ifndef EIGEN_USE_THREADS
#define EIGEN_USE_THREADS
#endif

#include <omp.h>
#include <unsupported/Eigen/CXX11/Tensor>

#include "wave/geometry_og/transformation.hpp"
#include "wave/odometry/odometry_types.hpp"
#include "wave/utils/math.hpp"

namespace wave {

struct TransformerParams {
    /// These parameters tradeoff between accuracy of transforms points and
    /// the number of high accuracy interpolated transforms calculated and stored

    /// Maximum difference between two subsequent transforms (radians)
    float delRTol = 1e-2f;
    /// Maximum difference between two subsequent angular velocities (radians/s)
    float delWTol = 1e-1f;
    /// Maximum difference between two subsequent linear velocities
    float delVTol = 1.f;

    /// Trajectories per scan
    uint32_t traj_resolution = 2;

    uint32_t n_scans = 2;
};

class Transformer {
 public:
    Transformer() = delete;
    explicit Transformer(TransformerParams params) : params(params) {}

    TransformerParams params;

    /// Prepares any required interpolated states
    void update(const std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> &trajectory,
                const std::vector<float> &stamps);

    /// Transforms feature tracks to start
    template<typename Scalar1, typename Scalar2>
    void transformTracksToStart(const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &tracks,
                                Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &tracks_transformed,
                                const uint32_t scan_idx);

    /** Points should be a 4xN tensor, xyz + timestamp
     * It is assumed the points are given in the sensor frame at the time that each one was observed.
     * They will be transformed to the sensor frame at the beginning of the first scan
     *
     * The timesteps for each point should be relative to the start of the scan
     *
     * @param points 2 dimensional tensor, 4 x N
     * @param points_transformed Dynamic Eigen Matrix, will be resized to 3 x N
     * @param which scan the points are from, used to select corresponding states/times
     * @param skip_point Whether or not to transform point. Provides speedup when not all feature points are used
     */
    template<typename Scalar1, typename Scalar2>
    void transformToStart(const Eigen::Tensor<Scalar1, 2> &points,
                          Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                          const uint32_t scan_idx,
                          const Vec<bool> * skip_point = nullptr);

    template<typename Scalar1, typename Scalar2>
    void transformToEnd(const Eigen::Tensor<Scalar1, 2> &points,
                        Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                        const uint32_t scan_idx);

    template<typename Scalar1, typename Scalar2>
    void transformToStart(const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &points,
                          Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                          const uint32_t scan_idx);

    template<typename Scalar1, typename Scalar2>
    void transformToEnd(const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &points,
                        Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                        const uint32_t scan_idx);

    template<typename Scalar1, typename Scalar2>
    void constantTransform(const uint32_t &fromScan,
                           const uint32_t &toScan,
                           const Eigen::Tensor<Scalar1, 2> &input,
                           Eigen::Tensor<Scalar2, 2> &output);

    template<typename Scalar1, typename Scalar2>
    void constantTransform(const uint32_t &fromScan,
                           const uint32_t &toScan,
                           const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &input,
                           Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &output);

 private:
    /// Contains the trajectories optimized over plus any extras required to meet difference criteria
    std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> aug_trajectories;
    /// Container for reusable differences.
    std::vector<PoseVelDiff, Eigen::aligned_allocator<PoseVelDiff>> differences;
    /// Given a scan index, what is the transform index in the trajectories container? Contains n_scans + 1 for the
    /// transform at the end of the trajectory.
    std::vector<uint32_t> scan_indices;
    /// Timestamp for each of the trajectories in the container.
    std::vector<float> traj_stamps;

    // With approximation used, each block of the interpolation factors is a scalar multiple of identity
    void calculateInterpolationFactors(const float &t1, const float &t2, const float &tau, Mat2 &candle, Mat2 &hat);
};
}

#include "impl/transformer_impl.hpp"

#endif  // WAVE_TRANSFORMER_HPP
