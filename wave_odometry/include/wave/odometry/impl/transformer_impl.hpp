#ifndef WAVE_TRANSFORMER_IMPL_HPP
#define WAVE_TRANSFORMER_IMPL_HPP

#include "wave/odometry/transformer.hpp"

namespace wave {

template<typename Scalar1, typename Scalar2>
void Transformer::transformTracksToStart(const Eigen::Matrix<Scalar1, -1, -1> &tracks,
                                         Eigen::Matrix<Scalar2, -1, -1> &tracks_transformed, const uint32_t scan_idx) {
    if (tracks_transformed.rows() != 3 || tracks_transformed.cols() != tracks.cols()) {
        tracks_transformed.resize(6, tracks.cols());
    }

    for (long i = 0; i < tracks.cols(); i++) {
        float pttime = tracks(6, i);
        if (pttime < 0) {
            throw std::out_of_range("point time not good");
        }
        float time = pttime + this->traj_stamps.at(this->scan_indices.at(scan_idx));
        auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
        auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
        if (index == this->traj_stamps.size()) {
            if (time - this->traj_stamps.back() < 0.001f) {
                index -= 2;
            } else {
                throw std::runtime_error("Invalid stamps in points 1");
            }
        }
        if (time < this->traj_stamps.at(index)) {
            if (index == 0) {
                throw std::runtime_error("Invalid stamps in points 2");
            }
            index--;
        }
        Mat2 hat, candle;
        this->calculateInterpolationFactors(
                this->traj_stamps.at(index), this->traj_stamps.at(index + 1), time, candle, hat);
        Vec6 tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                       candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                       candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
        Mat34 trans;
        T_TYPE::expMap1st(tan_vec, trans);
        auto &ref = this->aug_trajectories.at(index).pose.storage;
        tracks_transformed.template block<3, 1>(0, i).noalias() =
                (trans.block<3, 3>(0, 0) *
                 (ref.block<3, 3>(0, 0) * tracks.template block<3, 1>(0, i).template cast<double>() + ref.block<3, 1>(0, 3)) +
                 trans.block<3, 1>(0, 3)).template cast<Scalar2>();
        tracks_transformed.template block<3, 1>(3, i).noalias() = (trans.block<3, 3>(0, 0) *
                                                                  (ref.block<3, 3>(0, 0) * tracks.template block<3, 1>(3, i).template cast<double>())).template cast<Scalar2>();
    }
}

template<typename Scalar1, typename Scalar2>
void Transformer::transformToStart(const Eigen::Tensor<Scalar1, 2> &points,
                                   Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                                   const uint32_t scan_idx,
                                   const Vec<bool> * skip_point) {
    if (points_transformed.rows() != 3 || points_transformed.cols() != points.dimension(1)) {
        points_transformed.resize(3, points.dimension(1));
    }

    Eigen::Map<const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic>> pt(points.data(), points.dimension(0), points.dimension(1));
    auto &ptT = points_transformed;

//#pragma omp parallel for
    for (long i = 0; i < points.dimension(1); i++) {
        if (skip_point && skip_point->at(static_cast<unsigned long>(i))) {
            continue;
        }
        float time = points(3, i) + this->traj_stamps.at(this->scan_indices.at(scan_idx));
        auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
        auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
        if (index == this->traj_stamps.size()) {
            if (time - this->traj_stamps.back() < 0.001f) {
                index -= 2;
            } else {
                throw std::runtime_error("Invalid stamps in points 1");
            }
        }
        if (time < this->traj_stamps.at(index)) {
            if (index == 0) {
                throw std::runtime_error("Invalid stamps in points 2");
            }
            index--;
        }
        Mat2 hat, candle;
        this->calculateInterpolationFactors(
                this->traj_stamps.at(index), this->traj_stamps.at(index + 1), time, candle, hat);
        Vec6 tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                        candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                        candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
        Mat34 trans;
        T_TYPE::expMap1st(tan_vec, trans);
        auto &ref = this->aug_trajectories.at(index).pose.storage;
        ptT.template block<3, 1>(0, i).noalias() =
                (trans.block<3, 3>(0, 0) *
                (ref.block<3, 3>(0, 0) * pt.template block<3, 1>(0, i).template cast<double>() + ref.block<3, 1>(0, 3)) +
                trans.block<3, 1>(0, 3)).template cast<Scalar2>();
    }
}

template<typename Scalar1, typename Scalar2>
void Transformer::transformToStart(const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &pt,
                                   Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &ptT,
                                   const uint32_t scan_idx) {
    if (ptT.rows() != 3 || ptT.cols() != pt.cols()) {
        ptT.resize(3, pt.cols());
    }

    for (long i = 0; i < pt.cols(); i++) {
        float pttime = pt(3, i);
        if (pttime < 0) {
            throw std::out_of_range("point time not good");
        }
        float time = pt(3, i) + this->traj_stamps.at(this->scan_indices.at(scan_idx));
        auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
        auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
        if (index == this->traj_stamps.size()) {
            if (time - this->traj_stamps.back() < 0.001f) {
                index -= 2;
            } else {
                throw std::runtime_error("Invalid stamps in points 1");
            }
        }
        if (time < this->traj_stamps.at(index)) {
            if (index == 0) {
                throw std::runtime_error("Invalid stamps in points 2");
            }
            index--;
        }
        Mat2 hat, candle;
        this->calculateInterpolationFactors(
                this->traj_stamps.at(index), this->traj_stamps.at(index + 1), time, candle, hat);
        Vec6 tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                        candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                        candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
        Mat34 trans;
        T_TYPE::expMap1st(tan_vec, trans);
        auto &ref = this->aug_trajectories.at(index).pose.storage;
        ptT.template block<3, 1>(0, i).noalias() =
                (trans.block<3, 3>(0, 0) *
                 (ref.block<3, 3>(0, 0) * pt.template block<3, 1>(0, i).template cast<double>() + ref.block<3, 1>(0, 3)) +
                 trans.block<3, 1>(0, 3)).template cast<Scalar2>();
    }
}

template<typename Scalar1, typename Scalar2>
void Transformer::transformToEnd(const Eigen::Tensor<Scalar1, 2> &points,
                                 Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &points_transformed,
                                 const uint32_t scan_idx) {
    if (points_transformed.rows() != 3 || points_transformed.cols() != points.dimension(1)) {
        points_transformed.resize(3, points.dimension(1));
    }

    Eigen::Map<const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic>> pt(points.data(), points.dimension(0), points.dimension(1));
    auto &ptT = points_transformed;

//#pragma omp parallel for
    for (long i = 0; i < points.dimension(1); i++) {
        float time = points(3, i) + this->traj_stamps.at(this->scan_indices.at(scan_idx));
        auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
        auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
        if (time < this->traj_stamps.at(index)) {
            if (index == 0) {
                throw std::out_of_range("Invalid stamps in points");
            }
            index--;
        }
        Mat2 hat, candle;
        this->calculateInterpolationFactors(
          this->traj_stamps.at(index), this->traj_stamps.at(index + 1), time, candle, hat);
        Vec6 tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                        candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                        candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
        Mat34 trans;
        T_TYPE::expMap1st(tan_vec, trans);
        auto &ref = this->aug_trajectories.at(index).pose.storage;
        auto &final = this->aug_trajectories.back().pose.storage;
        ptT.template block<3, 1>(0, i).noalias() =
                (final.block<3, 3>(0, 0).transpose() *
          (trans.block<3, 3>(0, 0) *
             (ref.block<3, 3>(0, 0) * pt.template block<3, 1>(0, i).template cast<double>() + ref.block<3, 1>(0, 3)) +
           trans.block<3, 1>(0, 3) - final.block<3, 1>(0, 3))).template cast<Scalar2>();
    }
}

template<typename Scalar1, typename Scalar2>
void Transformer::transformToEnd(const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &pt,
                                 Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &ptT,
                                 const uint32_t scan_idx) {
    if (ptT.rows() != 3 || ptT.cols() != pt.cols()) {
        ptT.resize(3, pt.cols());
    }

//#pragma omp parallel for
    for (long i = 0; i < pt.cols(); i++) {
        float time = pt(3, i) + this->traj_stamps.at(this->scan_indices.at(scan_idx));
        auto idx = std::lower_bound(this->traj_stamps.begin(), this->traj_stamps.end(), time);
        auto index = static_cast<uint32_t>(idx - this->traj_stamps.begin());
        if (time < this->traj_stamps.at(index)) {
            if (index == 0) {
                throw std::out_of_range("Invalid stamps in points");
            }
            index--;
        }
        Mat2 hat, candle;
        this->calculateInterpolationFactors(
                this->traj_stamps.at(index), this->traj_stamps.at(index + 1), time, candle, hat);
        Vec6 tan_vec = hat(0, 1) * this->differences.at(index).hat_multiplier.block<6, 1>(6, 0) +
                        candle(0, 0) * this->differences.at(index).candle_multiplier.block<6, 1>(0, 0) +
                        candle(0, 1) * this->differences.at(index).candle_multiplier.block<6, 1>(6, 0);
        Mat34 trans;
        T_TYPE::expMap1st(tan_vec, trans);
        auto &ref = this->aug_trajectories.at(index).pose.storage;
        auto &final = this->aug_trajectories.back().pose.storage;
        ptT.template block<3, 1>(0, i).noalias() =
                (final.block<3, 3>(0, 0).transpose() *
                 (trans.block<3, 3>(0, 0) *
                  (ref.block<3, 3>(0, 0) * pt.template block<3, 1>(0, i).template cast<double>() + ref.block<3, 1>(0, 3)) +
                  trans.block<3, 1>(0, 3) - final.block<3, 1>(0, 3))).template cast<Scalar2>();
    }
}

template<typename Scalar1, typename Scalar2>
void Transformer::constantTransform(const uint32_t &fromScan,
                                    const uint32_t &toScan,
                                    const Eigen::Tensor<Scalar1, 2> &input,
                                    Eigen::Tensor<Scalar2, 2> &output) {
    output.resize(3, input.dimension(1));

    const Mat3 RtT =
      this->aug_trajectories.at(this->scan_indices.at(toScan)).pose.storage.block<3, 3>(0, 0).transpose();
    const Mat3 Rf =
      this->aug_trajectories.at(this->scan_indices.at(fromScan)).pose.storage.block<3, 3>(0, 0);

    const Vec3 Tt =
      this->aug_trajectories.at(this->scan_indices.at(toScan)).pose.storage.block<3, 1>(0, 3);
    const Vec3 Tf =
      this->aug_trajectories.at(this->scan_indices.at(fromScan)).pose.storage.block<3, 1>(0, 3);

    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = RtT * Rf;
    const Vec3 T = RtT * (Tf - Tt);

    Eigen::TensorMap<Eigen::Tensor<const double, 1>> R1(R.data(), 3);
    Eigen::TensorMap<Eigen::Tensor<const double, 1>> R2(R.data() + 3, 3);
    Eigen::TensorMap<Eigen::Tensor<const double, 1>> R3(R.data() + 6, 3);
    Eigen::TensorMap<Eigen::Tensor<const double, 2>> TT(T.data(), {3, 1});

    Eigen::array<ptrdiff_t, 1> dims({0});
    Eigen::array<long, 2> bcast({1, input.dimension(1)});

    output =
            (input.template cast<double>().convolve(R1, dims).concatenate(input.template cast<double>().convolve(R2, dims), 0).concatenate(input.template cast<double>().convolve(R3, dims), 0) +
      TT.broadcast(bcast)).template cast<Scalar2>();
}

template<typename Scalar1, typename Scalar2>
void Transformer::constantTransform(const uint32_t &fromScan,
                                    const uint32_t &toScan,
                                    const Eigen::Matrix<Scalar1, Eigen::Dynamic, Eigen::Dynamic> &input,
                                    Eigen::Matrix<Scalar2, Eigen::Dynamic, Eigen::Dynamic> &output) {
    output.resize(3, input.cols());

    const Mat3 RtT =
            this->aug_trajectories.at(this->scan_indices.at(toScan)).pose.storage.block<3, 3>(0, 0).transpose();
    const Mat3 Rf =
            this->aug_trajectories.at(this->scan_indices.at(fromScan)).pose.storage.block<3, 3>(0, 0);

    const Vec3 Tt =
            this->aug_trajectories.at(this->scan_indices.at(toScan)).pose.storage.block<3, 1>(0, 3);
    const Vec3 Tf =
            this->aug_trajectories.at(this->scan_indices.at(fromScan)).pose.storage.block<3, 1>(0, 3);

    const Mat3 R = RtT * Rf;
    const Vec3 T = RtT * (Tf - Tt);

//#pragma omp parallel for
    for (uint32_t i = 0; i < input.cols(); ++i) {
        output.template block<3,1>(0,i).noalias() = (R * input.template block<3,1>(0,i).template cast<double>() + T).template cast<Scalar2>();
    }
}
}

#endif  // WAVE_TRANSFORMER_HPP
