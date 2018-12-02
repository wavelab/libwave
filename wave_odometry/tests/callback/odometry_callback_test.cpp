/**
 * Unit tests for the odometry ceres callback
 */
#include <matplotlibcpp.h>

#include "wave/odometry/odometry_callback.hpp"
#include "wave/wave_test.hpp"

namespace wave {

// Fixture to initialize objects used in callback
class CallbackFixture : public testing::Test {
 protected:
    CallbackFixture() {}

    virtual ~CallbackFixture() {
        delete transformer;
        delete cb;
    }

    virtual void SetUp() {
        total_states = (traj_resolution - 1) * n_scans + 1;
        traj.resize(total_states);
        traj_stamps.resize(traj.size());
        for (uint32_t i = 0; i < traj_stamps.size(); ++i) {
            traj_stamps.at(i) = (float) (n_scans) * ((float) (i) / (float) (traj_stamps.size() - 1));
        }

        feat_pts.resize(this->n_scans);
        feat_ptsT.resize(this->n_scans);
        ptT_jacobians.resize(this->n_features);
        for (uint32_t i = 0; i < this->ptT_jacobians.size(); ++i) {
            ptT_jacobians.at(i).resize(n_scans);
        }

        auto traj_time = this->traj_stamps.begin() + 1;

        for (uint32_t i = 0; i < n_scans; ++i) {
            scan_stamps.emplace_back((float) i);
            feat_pts.at(i).resize(n_features);
            feat_ptsT.at(i).resize(n_features);
            
            for (uint32_t j = 0; j < n_features; ++j) {
                feat_pts.at(i).at(j) = Eigen::Tensor<float, 2>(4, n_pts);

                for (uint32_t k = 0; k < n_pts; ++k) {
                    Eigen::array<int, 2> offsets = {0, static_cast<int>(k)};
                    Eigen::array<int, 2> extents = {3, 1};
                    feat_pts.at(i).at(j).slice(offsets, extents).setRandom();
                    float stamp = static_cast<float>(((float) k / 5000.0));
                    feat_pts.at(i).at(j)(3, k) = stamp;
                    if (stamp > *traj_time) {
                        this->crossover.emplace_back(5000 * i + k);
                        ++traj_time;
                    }
                }
                auto &ref = feat_pts.at(i).at(j);
                Eigen::Map<MatXf> map(ref.data(), ref.dimension(0), ref.dimension(1));
                feat_ptsT.at(i).at(j) = map.block(0, 0, 3, map.cols()).cast<double>();
                ptT_jacobians.at(j).at(i).resize(n_states);
            }
        }

        TransformerParams transformerParams;
        transformerParams.traj_resolution = traj_resolution;
        transformer = new Transformer(transformerParams);

        cb = new OdometryCallback(&feat_pts, nullptr, nullptr,
                &feat_ptsT, nullptr, nullptr,
                &traj, &ptT_jacobians, &jac_stamps, &traj_stamps, &scan_stamps, transformer);
    }

    const unsigned int n_scans = 2;
    const unsigned int n_features = 2;
    const unsigned int n_states = 4;
    const unsigned int traj_resolution = 3;
    unsigned int total_states;
    const unsigned int n_pts = 5000;

    std::vector<unsigned long> crossover;
    Vec<VecE<Eigen::Tensor<float, 2>>> feat_pts;
    Vec<VecE<MatX>> feat_ptsT;
    VecE<PoseVel> traj;
    Vec<Vec<VecE<MatX>>> ptT_jacobians;
    Vec<Vec<float>> jac_stamps;
    Vec<float> traj_stamps, scan_stamps;
    Transformer *transformer;

    OdometryCallback *cb;
};

TEST_F(CallbackFixture, OdometryCallback) {}

TEST_F(CallbackFixture, PrepareForEvaluationFF) {
    cb->PrepareForEvaluation(false, false);
}

TEST_F(CallbackFixture, PrepareForEvaluationFT) {
    cb->PrepareForEvaluation(false, true);
}

TEST_F(CallbackFixture, PrepareForEvaluationTF) {
    cb->PrepareForEvaluation(true, false);
}

TEST_F(CallbackFixture, PrepareForEvaluationTT) {
    cb->PrepareForEvaluation(true, true);
}

TEST_F(CallbackFixture, JacobianTest) {
    const double step = std::sqrt(std::numeric_limits<double>::epsilon());
    // fill analytic Jacobians
    cb->PrepareForEvaluation(true, false);

    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> ptT_jacobians_num;
    ptT_jacobians_num.resize(n_scans);
    Vec6 diff;

    Eigen::Tensor<double, 2> coljac;

    for (uint32_t i = 0; i < this->n_scans; ++i) {
        ptT_jacobians_num.at(i).resize(this->n_features);
        for (uint32_t j = 0; j < this->n_features; ++j) {
            ptT_jacobians_num.at(i).at(j).resize(2 * total_states);
            for (uint32_t k = 0; k < this->total_states; ++k) {
                ptT_jacobians_num.at(i).at(j).at(2 * k) =
                  Eigen::Tensor<double, 3>(3, 12, this->feat_pts.at(i).at(j).dimension(1));
                ptT_jacobians_num.at(i).at(j).at(2 * k).setZero();
                ptT_jacobians_num.at(i).at(j).at(2 * k + 1) =
                  Eigen::Tensor<double, 3>(3, 6, this->feat_pts.at(i).at(j).dimension(1));
                for (uint32_t dim_idx = 0; dim_idx < 6; ++dim_idx) {
                    diff.setZero();
                    diff(dim_idx) = step;
                    // First do the pose portion
                    this->traj.at(k).pose.setIdentity();
                    this->traj.at(k).pose.manifoldPlus(diff);
                    this->transformer->update(this->traj, this->traj_stamps);
                    this->transformer->transformToStart(this->feat_pts.at(i).at(j), this->feat_ptsT.at(i).at(j), i);

                    Eigen::array<int, 2> offsets = {0, 0};
                    Eigen::array<int, 2> extents = {3, static_cast<int>(this->feat_pts.at(i).at(j).dimension(1))};

                    {
                        auto &ref = this->feat_ptsT.at(i).at(j);
                        Eigen::TensorMap<Eigen::Tensor<double, 2>> tmap(ref.data(), ref.rows(), ref.cols());

                        coljac = (tmap - this->feat_pts.at(i).at(j).cast<double>().slice(offsets, extents)) /
                                 step;

                        ptT_jacobians_num.at(i).at(j).at(2 * k).chip(dim_idx, 1) = coljac;
                    }

                    // now do velocity portion
                    this->traj.at(k).pose.setIdentity();
                    this->traj.at(k).vel(dim_idx) = step;

                    this->transformer->update(this->traj, this->traj_stamps);
                    this->transformer->transformToStart(this->feat_pts.at(i).at(j), this->feat_ptsT.at(i).at(j), i);

                    {
                        auto &ref = this->feat_ptsT.at(i).at(j);
                        Eigen::TensorMap<Eigen::Tensor<double, 2>> tmap(ref.data(), ref.rows(), ref.cols());

                        coljac = (tmap - this->feat_pts.at(i).at(j).cast<double>().slice(offsets, extents)) /
                                 step;

                        ptT_jacobians_num.at(i).at(j).at(2 * k + 1).chip(dim_idx, 1) = coljac;
                    }

                    this->traj.at(k).vel.setZero();
                }
            }
            /// At this point the numerical jacobians of each transformed point wrt each state should be calculated.
            /// The feature index does not matter, only the scan index and crossover points

            Vec<Vec<double>> max_error_vector(4);
            for (auto &e_vec : max_error_vector) {
                e_vec.clear();
            }
            int failure_counter = 0;
            for (uint32_t pt_idx = 0; pt_idx < this->n_pts; ++pt_idx) {
                float time = (float)(i) + this->feat_pts.at(i).at(j)(3, pt_idx);
                auto state_iter = std::lower_bound(
                  this->traj_stamps.begin(), this->traj_stamps.end(), time);
                auto state_offset = static_cast<uint32_t>(state_iter - this->traj_stamps.begin());

                if(*state_iter >= time && state_offset > 0) {
                    --state_offset;
                }

                Eigen::Tensor<int, 2> failures;

                for (uint32_t stat_num = 0; stat_num < 4; ++stat_num) {
                    MatX analytic_JT;
                    if (stat_num % 2 == 0) {
                        failures.resize(3, 12);
                        analytic_JT.resize(6, 12);
                        analytic_JT.setZero();
                    } else {
                        failures.resize(3, 6);
                        analytic_JT.resize(6,6);
                    }
                    Eigen::Tensor<double, 2> numerical_jacobian;
                    numerical_jacobian = ptT_jacobians_num.at(i).at(j).at(state_offset*2 + stat_num).chip(pt_idx, 2);

                    auto &time_vec = this->jac_stamps.at(state_offset);
                    auto iter = std::upper_bound(time_vec.begin(), time_vec.end(), time);

                    float T1, T2;

                    if (iter == time_vec.end()) {
                        T2 = *(--iter);
                        T1 = *(--iter);
                    } else {
                        T2 = *iter;
                        T1 = *(--iter);
                    }

                    double w1 = (T2 - time) / (T2 - T1);
                    double w2 = 1. - w1;

                    auto jac_offset = static_cast<uint32_t>(iter - time_vec.begin());

                    Eigen::Tensor<double, 2> numeric_jacobian = ptT_jacobians_num.at(i).at(j).at(state_offset*2 + stat_num).chip(pt_idx, 2);
                    MatX numerical = Eigen::Map<MatX>(numeric_jacobian.data(), numeric_jacobian.dimension(0), numeric_jacobian.dimension(1));

                    analytic_JT.block<6,6>(0,0) = w1 * this->ptT_jacobians.at(state_offset).at(stat_num).at(jac_offset) +
                            w2 * this->ptT_jacobians.at(state_offset).at(stat_num).at(jac_offset + 1);

                    MatX ptT_jacobian(3,6);
                    Vec3 pt = this->feat_ptsT.at(i).at(j).block<3,1>(0,pt_idx).cast<double>();
                    ptT_jacobian << 0, pt(2), -pt(1), 1, 0, 0,
                                   -pt(2), 0, pt(0), 0, 1, 0,
                                    pt(1), -pt(0), 0, 0, 0, 1;

                    MatX analytic_jacobian(3, numerical.cols());
                    analytic_jacobian.setZero();
                    analytic_jacobian.block<3,6>(0,0) = ptT_jacobian * analytic_JT.block<6,6>(0,0);

                    MatX error = numerical - analytic_jacobian;

                    // due to single precision floating point, threshold is relatively large
                    double max_error = error.cwiseAbs().maxCoeff();
                    max_error_vector.at(stat_num).emplace_back(max_error);
                    if (max_error > 1e-3) {
                        failure_counter++;
                    }
                }
            }
            // This was used to check the effect of interpolating the jacobians
//            matplotlibcpp::plot(max_error_vector.at(0));
//            matplotlibcpp::show(true);
//            matplotlibcpp::plot(max_error_vector.at(1));
//            matplotlibcpp::show(true);
//            matplotlibcpp::plot(max_error_vector.at(2));
//            matplotlibcpp::show(true);
//            matplotlibcpp::plot(max_error_vector.at(3));
//            matplotlibcpp::show(true);
            EXPECT_EQ(failure_counter, 0);
        }
    }
}
}
