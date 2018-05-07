#include "wave/wave_test.hpp"
#include "wave/odometry/transformer.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave{

TEST(Transformer, Transformer) {
    TransformerParams params;
    Transformer transformer(params);
}

/// Just test no segfaults when interpolating transform
TEST(Transformer, update) {
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 acc;
    acc << 0, 0, 1.5, 3, 0, 0;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(trajectory.at(i-1).vel * 0.1);
        }
        trajectory.at(i).vel = static_cast<double>(i) * 0.1 * acc;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);
}

/// Each scan consists of 100 points sampled from a circle
TEST(Transformer, transformToStart) {
    std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> trajectory;
    std::vector<float> stamps;

    TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    Vec6 acc;
    acc << 0, 0, 1.5, 3, 0, 0;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(trajectory.at(i-1).vel * 0.1);
        }
        trajectory.at(i).vel = static_cast<double>(i) * 0.1 * acc;
    }
    Transformer transformer(params);
    transformer.update(trajectory, stamps);

    Eigen::Tensor<float, 2> scan(4, 100);
}

/// Given a set of points in a circle, this should stretch/transform it smoothly across trajectory
/// Sanity check
TEST(Transformer, transformViz) {

}

}
