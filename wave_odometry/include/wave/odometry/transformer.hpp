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

#include "wave/odometry/odometry_types.hpp"
#include "wave/geometry/transformation.hpp"

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
    void update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory);

    /// Points should be a 4xN tensor, xyz + tick fraction
    void transformToStart(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset);
    void transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset);
 private:
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> aug_trajectories;
    std::vector<TrajDifference, Eigen::aligned_allocator<TrajDifference>> differences;
    std::vector<float> indices;

};

}

#endif //WAVE_TRANSFORMER_HPP
