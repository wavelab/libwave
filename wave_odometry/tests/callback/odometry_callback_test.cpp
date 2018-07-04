/**
 * Unit tests for the odometry ceres callback
 */

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
        total_states = (traj_resolution - 1)*n_scans + 1;
        traj.resize(total_states);
        traj_stamps.resize(traj.size());
        for(uint32_t i = 0; i < traj_stamps.size(); ++i) {
            traj_stamps.at(i) = (float)(n_scans) * ((float)(i)/(float)(traj_stamps.size() - 1));
        }

        feat_pts.resize(this->n_scans);
        feat_ptsT.resize(this->n_scans);
        ptT_jacobians.resize(this->n_scans);
        for(uint32_t i = 0; i < n_scans; ++i) {
            feat_pts.at(i).resize(n_features);
            feat_ptsT.at(i).resize(n_features);
            ptT_jacobians.at(i).resize(n_features);
            for(uint32_t j = 0; j < n_features; ++j) {
                feat_pts.at(i).at(j) = Eigen::Tensor<float, 2>(4, n_pts);
                auto traj_time = this->traj_stamps.begin() + 1;
                for(uint32_t k = 0; k < n_pts; ++k) {
                    Eigen::array<int, 2> offsets = {0, static_cast<int>(k)};
                    Eigen::array<int, 2> extents = {3, 1};
                    feat_pts.at(i).at(j).slice(offsets, extents).setRandom();
                    float stamp = static_cast<float>(i + ((float)k/5000.0));
                    feat_pts.at(i).at(j)(3, k) = stamp;
                    if (stamp > *traj_time) {
                        this->crossover.emplace_back(k);
                        ++traj_time;
                    }
                }
                feat_ptsT.at(i).at(j) = feat_pts.at(i).at(j);
                ptT_jacobians.at(i).at(j).resize(n_states);
            }
        }

        transformer = new Transformer(TransformerParams());

        cb = new OdometryCallback(&feat_pts, &feat_ptsT, &traj, &ptT_jacobians, &traj_stamps, transformer);
    }

    const unsigned int n_scans = 2;
    const unsigned int n_features = 2;
    const unsigned int n_states = 4;
    const unsigned int traj_resolution = 3;
    unsigned int total_states;
    const unsigned int n_pts = 5000;

    std::vector<unsigned long> crossover;
    Vec<VecE<Eigen::Tensor<float, 2>>> feat_pts;
    Vec<VecE<Eigen::Tensor<float, 2>>> feat_ptsT;
    VecE<PoseVel> traj;
    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> ptT_jacobians;
    Vec<float> traj_stamps;
    Transformer *transformer;

    OdometryCallback *cb;
};

TEST_F(CallbackFixture, OdometryCallback) {
}

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
    const double step = sqrt(std::numeric_limits<double>::epsilon());
    // fill analytic Jacobians
    cb->PrepareForEvaluation(true, false);

    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> ptT_jacobians_num;
    ptT_jacobians_num.resize(n_scans);
    Vec6 diff;

    Eigen::Tensor<double, 2> coljac;

    for(uint32_t i = 0; i < this->n_scans; ++i) {
        ptT_jacobians_num.at(i).resize(this->n_features);
        for (uint32_t j = 0; j < this->n_features; ++j) {
            ptT_jacobians_num.at(i).at(j).resize(2 * total_states);
            for (uint32_t k = 0; k < this->total_states; ++k) {
                ptT_jacobians_num.at(i).at(j).at(2*k) = Eigen::Tensor<double, 3>(3, 12, this->feat_pts.at(i).at(j).dimension(1));
                ptT_jacobians_num.at(i).at(j).at(2*k).setZero();
                ptT_jacobians_num.at(i).at(j).at(2*k + 1) = Eigen::Tensor<double, 3>(3, 6, this->feat_pts.at(i).at(j).dimension(1));
                for (uint32_t dim_idx = 0; dim_idx < 6; ++dim_idx) {
                    diff.setZero();
                    diff(dim_idx) = step;
                    // First do the pose portion
                    this->traj.at(k).pose.setIdentity();
                    this->traj.at(k).pose.manifoldPlus(diff);
                    this->transformer->update(this->traj, this->traj_stamps);
                    this->transformer->transformToStart(this->feat_pts.at(i).at(j), this->feat_ptsT.at(i).at(j));

                    Eigen::array<int, 2> offsets = {0, 0};
                    Eigen::array<int, 2> extents = {3, static_cast<int>(this->feat_pts.at(i).at(j).dimension(1))};
                    coljac = (this->feat_ptsT.at(i).at(j) - this->feat_pts.at(i).at(j).slice(offsets, extents)).cast<double>() / step;

                    ptT_jacobians_num.at(i).at(j).at(2*k).chip(dim_idx, 1) = coljac;

                    //now do velocity portion
                    this->traj.at(k).pose.setIdentity();
                    this->traj.at(k).vel(dim_idx) = step;

                    this->transformer->update(this->traj, this->traj_stamps);
                    this->transformer->transformToStart(this->feat_pts.at(i).at(j), this->feat_ptsT.at(i).at(j));

                    coljac = (this->feat_ptsT.at(i).at(j) - this->feat_pts.at(i).at(j).slice(offsets, extents)).cast<double>() / step;

                    ptT_jacobians_num.at(i).at(j).at(2*k + 1).chip(dim_idx, 1) = coljac;

                    this->traj.at(k).vel.setZero();
                }
                Eigen::Tensor<double, 2> random_analytic = ptT_jacobians.at(i).at(j).at(k).chip(0, 0);
                Eigen::Map<MatX> map_analytic(random_analytic.data(), random_analytic.dimension(0), random_analytic.dimension(1));
                MatX debugview_analytic = map_analytic;

                Eigen::Tensor<double, 2> random_numerical = ptT_jacobians_num.at(i).at(j).at(k).chip(0, 0);
                Eigen::Map<MatX> map_numerical(random_numerical.data(), random_numerical.dimension(0), random_numerical.dimension(1));
                MatX debugview = map_numerical;

                // Each point is associated with 2 posevel states, so to compute error need to compare between the right
                // jacobians

                // the timestamps of the points are i + k/5000. Goes from 0 to 2 - 1/5000;
                // there are traj_res states per scan, counting the start and end states. 5 in total spaced evenly in
                // time.

                // Within each scan and feature type, there are 5000 points spaced evenly.

                for (uint32_t pt_idx = 0; pt_idx < this->n_pts; ++pt_idx) {
//                    auto error = ptT_jacobians_num.at(i).at(j).at()
                }

                Eigen::Tensor<double, 3> error = ptT_jacobians_num.at(i).at(j).at(k) - ptT_jacobians.at(i).at(j).at(k);

                Eigen::Tensor<double, 2> random_error = error.chip(0, 0);
                Eigen::Map<MatX> map_error(random_error.data(), random_error.dimension(0), random_error.dimension(1));
                MatX error_view = map_error;

                Eigen::Tensor<int, 3> failures = error.operator>(1e-6).cast<int>();

                std::vector<std::vector<uint32_t>> failure_idx;

                for (uint32_t pt = 0; pt < failures.dimension(2); ++pt) {
                    for (uint32_t col = 0; col < failures.dimension(1); ++col) {
                        for (uint32_t row = 0; row < failures.dimension(0); ++ row) {
                            if (failures(row, col, pt)) {
                                failure_idx.emplace_back(std::vector<uint32_t>({row, col, pt}));
                            }
                        }
                    }
                }

                Eigen::Tensor<double, 0> worst = error.maximum();
                Eigen::Tensor<int, 0> count = failures.sum();
                EXPECT_EQ(count(0), 0);
            }
        }
    }
}

}
