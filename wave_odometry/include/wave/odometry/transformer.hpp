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

#include "wave/geometry/transformation.hpp"
#include "wave/odometry/odometry_types.hpp"

namespace wave {

struct TransformerParams {
    /// Maximum difference between two subsequent transforms (radians)
    float delRTol = 1e-1f;
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
    Transformer(TransformerParams params) : params(params) {}

    TransformerParams params;

    /// Prepares any required interpolated states
    void update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory,
                const std::vector<float> &stamps);

    /** Points should be a 4xN tensor, xyz + timestamp
     * It is assumed the points are given in the sensor frame at the time that each one was observed.
     * They will be transformed to the sensor frame at the beginning of the scan
     *
     * The timesteps for each point should be relative to the start of the scan
     *
     * @param points 2 dimensional tensor, 4 x N
     * @param points_transformed 2 dimensional tensor, will be resized to 3 x N
     * @param scan index of scan
     */
    void transformToStart(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed,
                          const uint32_t &scan);
    void transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed,
                            const uint32_t &scan);

 private:
    /// Contains the trajectories optimized over plus any extras required to meet difference criteria
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> aug_trajectories;
    /// Container for reusable differences.
    std::vector<TrajDifference, Eigen::aligned_allocator<TrajDifference>> differences;
    /// Given a scan index, what is the transform index in the trajectories container?
    std::vector<uint32_t> scan_indices;
    /// Timestamp for each of the trajectories in the container.
    std::vector<float> traj_stamps;

    // With approximation used, each block of the interpolation factors is a scalar multiple of identity
    void calculateInterpolationFactors(const float &t1, const float &t2, const float &tau, Mat4 &candle, Mat4 &hat);
};
}

#endif  // WAVE_TRANSFORMER_HPP
