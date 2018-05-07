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
    Transformer(TransformerParams params = TransformerParams()) : params(params) {}

    TransformerParams params;

    /// Prepares any required interpolated states
    void update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory,
                const std::vector<float> &stamps);

    /** Points should be a 4xN tensor, xyz + tick fraction
     * It is assumed the points are given in the sensor frame at the time that each one was observed.
     * They will be transformed to the sensor frame at the beginning of the scan
     * @param points
     * @param points_transformed
     * @param scan_offset
     */
    void transformToStart(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset);
    void transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset);
 private:
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> aug_trajectories;
    std::vector<TrajDifference, Eigen::aligned_allocator<TrajDifference>> differences;
    std::vector<float> scan_stamps;
    std::vector<float> indices;

    // With approximation used, each block of the interpolation factors is a scalar multiple of identity
    void calculateInterpolationFactors(const float &t1, const float &t2, const float &tau, Mat4 &candle, Mat4 &hat);

};

}

#endif //WAVE_TRANSFORMER_HPP
