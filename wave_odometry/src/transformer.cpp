#include "wave/odometry/transformer.hpp"

namespace wave {

void Transformer::update(const std::vector<PoseVel, Eigen::aligned_allocator<PoseVel>> &trajectory,
                         const std::vector<float> &stamps) {
    this->aug_trajectories = trajectory;
    this->traj_stamps = stamps;
    this->scan_indices.resize((this->traj_stamps.size() - 1) / (this->params.traj_resolution - 1));
    uint32_t n = 0;
    for (auto &val : this->scan_indices) {
        val = n;
        n += this->params.traj_resolution - 1;
    }

    uint32_t i = 0;
    Mat2 hat, candle;
    while (i + 1 < this->traj_stamps.size()) {
        Vec6 pose_diff = this->aug_trajectories.at(i + 1).pose.manifoldMinus(this->aug_trajectories.at(i).pose);
        Vec6 vel_diff = this->aug_trajectories.at(i + 1).vel - this->aug_trajectories.at(i).vel;
        if (pose_diff.block<3, 1>(0, 0).cwiseAbs().sum() > this->params.delRTol ||
            vel_diff.block<3, 1>(3, 0).cwiseAbs().sum() > this->params.delVTol ||
            vel_diff.block<3, 1>(0, 0).cwiseAbs().sum() > this->params.delWTol) {
            float new_stamp = (this->traj_stamps.at(i + 1) + this->traj_stamps.at(i)) * 0.5f;
            this->calculateInterpolationFactors(
                    this->traj_stamps.at(i), this->traj_stamps.at(i + 1), new_stamp, candle, hat);

            PoseVel interp;
            T_TYPE::interpolateReduced(this->aug_trajectories.at(i).pose,
                                       this->aug_trajectories.at(i + 1).pose,
                                       this->aug_trajectories.at(i).vel,
                                       this->aug_trajectories.at(i + 1).vel,
                                       hat,
                                       candle,
                                       interp.pose,
                                       &(interp.vel));

            for (auto &idx : this->scan_indices) {
                if (this->traj_stamps.at(idx) > new_stamp) {
                    ++idx;
                }
            }
            this->aug_trajectories.insert(this->aug_trajectories.begin() + i + 1, interp);
            this->traj_stamps.insert(this->traj_stamps.begin() + i + 1, new_stamp);
        } else {
            ++i;
        }
    }
    // Now precalculate the differences between each transform
    this->differences.resize(this->traj_stamps.size() - 1);
    i = 0;
    for (auto &diff : this->differences) {
        diff.hat_multiplier.block<6, 1>(0, 0).setZero();
        diff.hat_multiplier.block<6, 1>(6, 0) = this->aug_trajectories.at(i).vel;
        diff.candle_multiplier.block<6, 1>(0, 0) =
                this->aug_trajectories.at(i + 1).pose.manifoldMinus(this->aug_trajectories.at(i).pose);
        diff.candle_multiplier.block<6, 1>(6, 0) =
                T_TYPE::SE3ApproxInvLeftJacobian(diff.candle_multiplier.block<6, 1>(0, 0)) *
                this->aug_trajectories.at(i + 1).vel;
        ++i;
    }
}

void Transformer::calculateInterpolationFactors(
        const float &t1, const float &t2, const float &tau, Mat2 &candle, Mat2 &hat) {
    float T1 = tau - t1;
    float T2 = t2 - tau;
    float dT = t2 - t1;
    float invT = 1.0f / dT;

    candle(0, 0) = (T1 * T1 * (4 * T1 - 3 * dT + 6 * T2)) * invT * invT * invT;
    candle(1, 0) = (6 * T1 * (T1 - dT + 2 * T2)) * invT * invT * invT;
    candle(0, 1) = -(T1 * T1 * (2 * T1 - 2 * dT + 3 * T2)) * invT * invT;
    candle(1, 1) = -(T1 * (3 * T1 - 4 * dT + 6 * T2)) * invT * invT;

    hat(0, 0) = 1.0f - candle(0, 0);
    hat(1, 0) = -candle(1, 0);
    hat(0, 1) = T1 - dT * candle(0, 0) - candle(0, 1);
    hat(1, 1) = 1.0f - dT * candle(1, 0) - candle(1, 1);
}

}
