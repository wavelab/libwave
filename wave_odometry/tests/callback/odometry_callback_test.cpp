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
        feat_pts.resize(this->n_scans);
        feat_ptsT.resize(this->n_scans);
        ptT_jacobians.resize(this->n_scans);
        for(uint32_t i = 0; i < n_scans; ++i) {
            feat_pts.at(i).resize(n_features);
            feat_ptsT.at(i).resize(n_features);
            ptT_jacobians.at(i).resize(n_features);
            for(uint32_t j = 0; j < n_features; ++j) {
                feat_pts.at(i).at(j) = Eigen::Tensor<float, 2>(4, n_pts);
                for(uint32_t k = 0; k < n_pts; ++k) {
                    Eigen::array<int, 2> offsets = {0, static_cast<int>(k)};
                    Eigen::array<int, 2> extents = {3, 1};
                    feat_pts.at(i).at(j).slice(offsets, extents).setRandom();
                    feat_pts.at(i).at(j)(3, k) = static_cast<float>(i + ((float)k/5000.0));
                }
                feat_ptsT.at(i).at(j) = feat_pts.at(i).at(j);
                ptT_jacobians.at(i).at(j).resize(n_states);
            }
        }

        traj.resize((traj_resolution - 1)*n_scans + 1);
        traj_stamps.resize(traj.size());

        for(uint32_t i = 0; i < traj_stamps.size(); ++i) {
            traj_stamps.at(i) = (float)(n_scans) * ((float)(i)/(float)(traj_stamps.size() - 1));
        }

        transformer = new Transformer(TransformerParams());

        cb = new OdometryCallback(&feat_pts, &feat_ptsT, &traj, &ptT_jacobians, &traj_stamps, transformer);
    }

    const unsigned int n_scans = 2;
    const unsigned int n_features = 2;
    const unsigned int n_states = 4;
    const unsigned int traj_resolution = 3;
    const unsigned int n_pts = 5000;

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
    const auto n_states = this->traj.size();
    // fill analytic Jacobians
    cb->PrepareForEvaluation(true, false);

    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> ptT_jacobians_num;
    ptT_jacobians_num.resize(n_scans);
    Vec6 diff;

    Eigen::Tensor<double, 2> coljac;

    for(uint32_t i = 0; i < this->n_scans; ++i) {
        ptT_jacobians_num.at(i).resize(this->n_features);
        for (uint32_t j = 0; j < this->n_features; ++j) {
            ptT_jacobians_num.at(i).at(j).resize(2 * n_states);
            for (uint32_t k = 0; k < this->n_states; ++k) {
                ptT_jacobians_num.at(i).at(j).at(2*k) = Eigen::Tensor<double, 3>(this->feat_pts.at(i).at(j).dimension(1), 3, 12);
                ptT_jacobians_num.at(i).at(j).at(2*k).setZero();
                ptT_jacobians_num.at(i).at(j).at(2*k + 1) = Eigen::Tensor<double, 3>(this->feat_pts.at(i).at(j).dimension(1), 3, 6);
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

                    ptT_jacobians_num.at(i).at(j).at(2*k).chip(dim_idx, 2).chip(0, 1) = coljac.chip(0, 0);
                    ptT_jacobians_num.at(i).at(j).at(2*k).chip(dim_idx, 2).chip(1, 1) = coljac.chip(1, 0);
                    ptT_jacobians_num.at(i).at(j).at(2*k).chip(dim_idx, 2).chip(2, 1) = coljac.chip(2, 0);

                    //now do velocity portion
                    this->traj.at(k).pose.setIdentity();
                    this->traj.at(k).vel(dim_idx) = step;

                    this->transformer->update(this->traj, this->traj_stamps);
                    this->transformer->transformToStart(this->feat_pts.at(i).at(j), this->feat_ptsT.at(i).at(j));

                    coljac = (this->feat_ptsT.at(i).at(j) - this->feat_pts.at(i).at(j).slice(offsets, extents)).cast<double>() / step;

                    ptT_jacobians_num.at(i).at(j).at(2*k + 1).chip(dim_idx, 2).chip(0, 1) = coljac.chip(0, 0);
                    ptT_jacobians_num.at(i).at(j).at(2*k + 1).chip(dim_idx, 2).chip(1, 1) = coljac.chip(1, 0);
                    ptT_jacobians_num.at(i).at(j).at(2*k + 1).chip(dim_idx, 2).chip(2, 1) = coljac.chip(2, 0);
                    this->traj.at(k).vel.setZero();
                }
                Eigen::Tensor<double, 2> random_analytic = ptT_jacobians.at(i).at(j).at(2*k).chip(0, 0);
                Eigen::Map<MatX> map_analytic(random_analytic.data(), random_analytic.dimension(0), random_analytic.dimension(1));
                MatX debugview_analytic = map_analytic;

                Eigen::Tensor<double, 2> random_numerical = ptT_jacobians_num.at(i).at(j).at(2*k).chip(0, 0);
                Eigen::Map<MatX> map_numerical(random_numerical.data(), random_numerical.dimension(0), random_numerical.dimension(1));
                MatX debugview = map_numerical;

                Eigen::Tensor<double, 3> error = ptT_jacobians_num.at(i).at(j).at(2*k) - ptT_jacobians.at(i).at(j).at(2*k);

                Eigen::Tensor<double, 2> random_error = error.chip(0, 0);
                Eigen::Map<MatX> map_error(random_error.data(), random_error.dimension(0), random_error.dimension(1));
                MatX error_view = map_error;

                Eigen::Tensor<int, 3> failures = error.operator>(1e-6).cast<int>();

                std::vector<int> failure_idx;
                unsigned long int stopping = failures.dimension(0) * failures.dimension(1) * failures.dimension(2);
                for(unsigned long int h = 0; h < stopping; ++h) {
                    if (*(failures.data() + h))
                        failure_idx.emplace_back(h);
                }

                Eigen::Tensor<double, 0> worst = error.maximum();
                Eigen::Tensor<int, 0> count = failures.sum();
                EXPECT_EQ(count(0), 0);
            }
        }
    }
}

}
