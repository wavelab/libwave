#include "wave/odometry/transformer.hpp"

namespace wave {

void Transformer::update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory,
                         const std::vector<float> &stamps) {
    this->aug_trajectories = trajectory;
    this->stamps = stamps;
    this->indices.resize(this->params.n_scans * (this->params.traj_resolution - 1) + 1);
    std::generate(this->indices.begin(), this->indices.end(), [n = 0] () mutable {return (float) n++;});

    uint32_t i = 0;
    Mat4 hat, candle;
    while (i + 1 < this->indices.size()) {
        Vec6 pose_diff = this->aug_trajectories.at(i+1).pose.manifoldMinus(this->aug_trajectories.at(i).pose);
        Vec6 vel_diff = this->aug_trajectories.at(i+1).vel - this->aug_trajectories.at(i).vel;
        if (pose_diff.block<3, 1>(0,0).cwiseAbs().sum() > this->params.delRTol ||
            vel_diff.block<3, 1>(3,0).cwiseAbs().sum() > this->params.delVTol ||
            vel_diff.block<3, 1>(0,0).cwiseAbs().sum() > this->params.delWTol) {

            this->calculateInterpolationFactors(this->stamps.at(i), this->stamps.at(i + 1),
                                                this->stamps.at(i+1) - this->stamps.at(i), candle, hat);

            Trajectory interp;
            T_TYPE::interpolate(this->aug_trajectories.at(i).pose,
                                this->aug_trajectories.at(i + 1).pose,
                                this->aug_trajectories.at(i).vel,
                                this->aug_trajectories.at(i + 1).vel,
                                hat, candle, interp.pose, &(interp.vel));

            this->aug_trajectories.insert(this->aug_trajectories.begin() + i, interp);
            this->stamps.insert(this->stamps.begin() + i, (this->stamps.at(i+1) + this->stamps.at(i)) * 0.5f);
            this->indices.insert(std::next(this->indices.begin(), i), (this->indices.at(i+1) - this->indices.at(i)) / 2.0f);
        } else {
            ++i;
        }
    }
    // Now precalculate the differences between each transform
    this->differences.resize(this->indices.size() - 1);
    i = 0;
    for(auto &diff : this->differences) {
        diff.hat_multiplier.block<6, 1>(0, 0).setZero();
        diff.hat_multiplier.block<6, 1>(6, 0) = this->aug_trajectories.at(i).vel;
        diff.candle_multiplier.block<6, 1>(0, 0) =
                this->aug_trajectories.at(i + 1).pose.manifoldMinus(this->aug_trajectories.at(i).pose);
        diff.candle_multiplier.block<6, 1>(6, 0) =
                T_TYPE::SE3ApproxInvLeftJacobian(diff.candle_multiplier.block<6, 1>(0, 0)) *
                        this->aug_trajectories.at(i + 1).vel;
    }
}

void Transformer::transformToStart(const Eigen::Tensor<float, 2> &points,
                                   Eigen::Tensor<float, 2> &points_transformed,
                                   int scan_offset) {
    points_transformed.resize(3, points.dimensions(1));

    //todo possibly look at tensorizing this, but may not help dt memory
    Mat4 hat, candle;
    Mat34f trans;
    Vec6f tan_vec;
    Eigen::Map<const VecXf> pt(points.data(), points.dimensions(0), points.dimensions(1));
    Eigen::Map<VecXf> ptT(points_transformed.data(), points_transformed.dimension(0), points_transformed.dimension(1));
    for (long i = 0; i < points.dimensions(1); i++) {
        auto idx = std::lower_bound(this->stamps.begin(), this->stamps.end(), points(3,i));
        this->calculateInterpolationFactors(*idx, *(idx + 1), points(3, i), candle, hat);
        uint32_t index = idx - this->stamps.begin();
        tan_vec = hat(0,1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                  candle(0,0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                  candle(0,1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
                T_TYPE::expMap(tan_vec, trans);
        ptT.block<3, 1>(0, i).noalias() = trans.block<3,3>(0,0) * pt.block<3, 1>(0, i) + trans.block<3, 1>(0,3);
    }
}

void Transformer::transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed, int scan_offset) {}

void Transformer::calculateInterpolationFactors(const float &t1, const float &t2, const float &tau, Mat4 &candle,
                                                Mat4 &hat) {
    float T1 = tau - t1;
    float T2 = t2 - tau;
    float dT = t2 - t1;
    float invT = 1.0f/dT;

    candle(0,0) = (T1*T1*(4*T1 - 3*dT + 6*T2)) * invT * invT * invT;
    candle(1,0) = (6*T1*(T1 - dT + 2*T2))* invT * invT * invT;
    candle(0,1) = -(T1*T1*(2*T1 - 2*dT + 3*T2)) * invT * invT;
    candle(1,1) = -(T1*(3*T1 - 4*dT + 6*T2))* invT * invT;

    hat(0,0) = 1.0f - candle(0,0);
    hat(1,0) = - candle(1,0);
    hat(0,1) = T1 - dT * candle(0,0) - candle(0,1);
    hat(1,1) = 1.0f - dT * candle(1,0) - candle(1,1);
}
}