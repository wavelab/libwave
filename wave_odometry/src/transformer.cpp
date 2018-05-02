#include "wave/odometry/transformer.hpp"

namespace wave {

void Transformer::update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory) {
    this->aug_trajectories = trajectory;
    this->indices.resize(this->params.n_scans * (this->params.traj_resolution - 1) + 1);
    std::generate(this->indices.begin(), this->indices.end(), [n = 0] () mutable {return (float) n++;});

    uint32_t i = 0;
    while (i + 1 < this->indices.size()) {
        Vec6 pose_diff = this->aug_trajectories.at(i+1).pose.manifoldMinus(this->aug_trajectories.at(i));
        Vec6 vel_diff = this->aug_trajectories.at(i+1).vel - this->aug_trajectories.at(i).vel;
        if (pose_diff.block<3, 1>(0,0).abs().sum() > this->params.delRTol ||
            vel_diff.block<3, 1>(3,0).abs().sum() > this->params.delVTol ||
            vel_diff.block<3, 1>(0,0).abs().sum() > this->params.delWTol) {
            T_TYPE interp;
            T_TYPE::interpolate(this->aug_trajectories.at(i).pose,
                                this->aug_trajectories.at(i + 1).pose,
                                this->aug_trajectories.at(i).vel,
                                this->aug_trajectories.at(i + 1).vel,
                                hat, candle, interp);
            this->aug_trajectories.insert(std::next(this->aug_trajectories.begin(), i), interp);
            this->indices.insert(std::next(this->indices.begin(), i), (this->indices.at(i+1) - this->indices.at(i)) / 2.0f);
        } else {
            ++i;
        }
    }
    // Now precalculate the differences between each transform

}

void Transformer::transformToStart(const Eigen::Tensor<float, 2> &points,
                                   Eigen::Tensor<float, 2> &points_transformed,
                                   int scan_offset) {}

void Transformer::transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset) {}
}