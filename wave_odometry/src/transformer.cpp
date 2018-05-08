#include "wave/odometry/transformer.hpp"

namespace wave {

void Transformer::update(const std::vector<Trajectory, Eigen::aligned_allocator<Trajectory>> &trajectory,
                         const std::vector<float> &stamps) {
    this->aug_trajectories = trajectory;
    this->traj_stamps = stamps;
    this->scan_indices.resize(this->params.n_scans);
    uint32_t n = 0;
    for (auto &val : this->scan_indices) {
        val = n;
        n += this->params.traj_resolution - 1;
    }

    uint32_t i = 0;
    Mat4 hat, candle;
    while (i + 1 < this->traj_stamps.size()) {
        Vec6 pose_diff = this->aug_trajectories.at(i + 1).pose.manifoldMinus(this->aug_trajectories.at(i).pose);
        Vec6 vel_diff = this->aug_trajectories.at(i + 1).vel - this->aug_trajectories.at(i).vel;
        if (pose_diff.block<3, 1>(0, 0).cwiseAbs().sum() > this->params.delRTol ||
            vel_diff.block<3, 1>(3, 0).cwiseAbs().sum() > this->params.delVTol ||
            vel_diff.block<3, 1>(0, 0).cwiseAbs().sum() > this->params.delWTol) {
            float new_stamp = (this->traj_stamps.at(i + 1) + this->traj_stamps.at(i)) * 0.5f;
            this->calculateInterpolationFactors(
              this->traj_stamps.at(i), this->traj_stamps.at(i + 1), new_stamp, candle, hat);

            Trajectory interp;
            T_TYPE::interpolate(this->aug_trajectories.at(i).pose,
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
        diff.hat_multiplier.block<6, 1>(6, 0) = this->aug_trajectories.at(i).vel.cast<float>();
        diff.candle_multiplier.block<6, 1>(0, 0) =
          this->aug_trajectories.at(i + 1).pose.manifoldMinus(this->aug_trajectories.at(i).pose).cast<float>();
        diff.candle_multiplier.block<6, 1>(6, 0) =
          T_TYPE::SE3ApproxInvLeftJacobian(diff.candle_multiplier.block<6, 1>(0, 0)).cast<float>() *
          this->aug_trajectories.at(i + 1).vel.cast<float>();
        ++i;
    }

    /** At this point, all of the transforms are with respect to a common frame, now, in order to save a transform
     * inversion and multiplication later, all transforms be recast to be
     * with respect to the transform at the start of the scan
     * DANGER DANGER, this means that the states are no longer proper, because the twist doesn't get transformed, but it
     * doesn't matter because the differences have already been computed.
     */
    i = 0;
    for (uint32_t j = 0; j < this->aug_trajectories.size(); j++) {
        this->aug_trajectories.at(j).pose =
          this->aug_trajectories.at(this->scan_indices.at(i)).pose.transformInverse() *
          this->aug_trajectories.at(j).pose;
        if (j + 1 == this->scan_indices.at(i)) {
            ++i;
        }
    }
}

void Transformer::transformToStart(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed) {
    points_transformed.resize(3, points.dimensions().at(1));

    Eigen::Map<const MatXf> pt(points.data(), points.dimension(0), points.dimension(1));
    Eigen::Map<MatXf> ptT(points_transformed.data(), points_transformed.dimension(0), points_transformed.dimension(1));

#pragma omp parallel
    {
        long threadnum = omp_get_thread_num();
        long numthreads = omp_get_num_threads();
        long low = points.dimension(1) * threadnum / numthreads;
        long high = points.dimension(1) * (threadnum + 1) / numthreads;
        for (long i = low; i < high; i++) {
            float time = points(3, i);
            auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
            auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
            if (time < this->traj_stamps.at(index)) {
                if (index == 0) {
                    throw std::out_of_range("Invalid stamps in points");
                }
                index--;
            }
            Mat4 hat, candle;
            this->calculateInterpolationFactors(
              this->traj_stamps.at(index), this->traj_stamps.at(index + 1), points(3, i), candle, hat);
            Vec6f tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                            candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                            candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
            Mat34f trans;
            T_TYPE::expMap1st(tan_vec, trans);
            auto &ref = this->aug_trajectories.at(index).pose.storage;
            ptT.block<3, 1>(0, i).noalias() =
              trans.block<3, 3>(0, 0) *
                (ref.block<3, 3>(0, 0).cast<float>() * pt.block<3, 1>(0, i) + ref.block<3, 1>(0, 3).cast<float>()) +
              trans.block<3, 1>(0, 3);
        }
    }
}

void Transformer::transformToEnd(const Eigen::Tensor<float, 2> &points, Eigen::Tensor<float, 2> &points_transformed) {
    points_transformed.resize(3, points.dimensions().at(1));

    Eigen::Map<const MatXf> pt(points.data(), points.dimension(0), points.dimension(1));
    Eigen::Map<MatXf> ptT(points_transformed.data(), points_transformed.dimension(0), points_transformed.dimension(1));

#pragma omp parallel
    {
        long threadnum = omp_get_thread_num();
        long numthreads = omp_get_num_threads();
        long low = points.dimension(1) * threadnum / numthreads;
        long high = points.dimension(1) * (threadnum + 1) / numthreads;
        for (long i = low; i < high; i++) {
            float time = points(3, i);
            auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
            auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
            if (time < this->traj_stamps.at(index)) {
                if (index == 0) {
                    throw std::out_of_range("Invalid stamps in points");
                }
                index--;
            }
            Mat4 hat, candle;
            this->calculateInterpolationFactors(
              this->traj_stamps.at(index), this->traj_stamps.at(index + 1), points(3, i), candle, hat);
            Vec6f tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                            candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                            candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
            Mat34f trans;
            T_TYPE::expMap1st(tan_vec, trans);
            auto &ref = this->aug_trajectories.at(index).pose.storage;
            ptT.block<3, 1>(0, i).noalias() =
              ref.block<3, 3>(0, 0).transpose().cast<float>() *
              (trans.block<3, 3>(0, 0).transpose() * (pt.block<3, 1>(0, i) - trans.block<3, 1>(0, 3)) -
               ref.block<3, 1>(0, 3).cast<float>());
        }
    }
}

void Transformer::calculateInterpolationFactors(
  const float &t1, const float &t2, const float &tau, Mat4 &candle, Mat4 &hat) {
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