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

}
