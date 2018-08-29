#include "wave/wave_test.hpp"
#include "wave/utils/utils.hpp"
#include "wave/optimization/ceres/tiny/robust_line_fit.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace wave {

namespace {

// Fixture to perform setup
class RobustLineFitFixture : public testing::Test {
 protected:
    RobustLineFitFixture() {}

    virtual ~RobustLineFitFixture() {}

    virtual void SetUp() {
        local_delta = Vec4::Zero();
        global_line.setRandom();
        global_line.block<3,1>(0,0).normalize();
        Pts.resize(13);
        for (uint32_t i = 0; i < Pts.size(); ++i) {
            Pts.at(i).setRandom();
            pts.emplace_back(Pts.at(i).data());
        }
    }

    Vec6 global_line;
    Vec4 local_delta;
    double robust_param = 200;
    std::vector<const float *> pts;
    VecE<Vec3f> Pts;
};

}

TEST_F(RobustLineFitFixture, InvalidCreation) {
    pts.clear();
    EXPECT_THROW(RobustLineFit(global_line.data(), local_delta.data(), pts, robust_param), std::runtime_error);
}

TEST_F(RobustLineFitFixture, ValidCreation) {
    EXPECT_NO_THROW(RobustLineFit(global_line.data(), local_delta.data(), pts, robust_param));
    RobustLineFit line_fit(global_line.data(), local_delta.data(), pts, robust_param);

    EXPECT_EQ(line_fit.NumResiduals(), static_cast<int>(pts.size()));
    EXPECT_EQ(line_fit.NUM_PARAMETERS, 4);
}

TEST_F(RobustLineFitFixture, JacobianTest) {
    RobustLineFit line_fit(global_line.data(), local_delta.data(), pts, robust_param);

    double step_size = 1e-8;
    Eigen::Matrix<double, Eigen::Dynamic, 4> analytical_jacobian(this->pts.size(), 4);
    Eigen::Matrix<double, Eigen::Dynamic, 4> numerical_jacobian(this->pts.size(), 4);

    Eigen::Matrix<double, Eigen::Dynamic, 1> zero_residual(this->pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> perturbed_residual(this->pts.size());
    Vec4 perturbation = Vec4::Zero();

    line_fit(local_delta.data(), zero_residual.data(), analytical_jacobian.data());

    for (uint32_t i = 0; i < numerical_jacobian.cols(); ++i) {
        perturbation.setZero();
        perturbation(i) = step_size;
        line_fit(perturbation.data(), perturbed_residual.data(), nullptr);

        numerical_jacobian.col(i) = (perturbed_residual - zero_residual) / step_size;
    }

    MatX error = analytical_jacobian - numerical_jacobian;
    double frob_error = error.norm();

    EXPECT_NEAR(frob_error, 0.0, 1e-6);
}

TEST_F(RobustLineFitFixture, SampleProblemNoNoise) {
    // put some points along a line
    Vec3f normal;
    normal.setRandom();
    normal.normalize();
    VecE<Vec3f> points(9);
    Vec3f datum = Vec3f::Zero();
    this->pts.clear();
    for (uint32_t i = 0; i < points.size(); ++i) {
        points.at(i) = datum + (float)(i) * 0.25 * normal;
        this->pts.emplace_back(points.at(i).data());
    }

    RobustLineFit line_fit(global_line.data(), local_delta.data(), pts, robust_param);

    auto cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto ptr : this->pts) {
        pcl::PointXYZ new_pt;
        new_pt.x = ptr[0];
        new_pt.y = ptr[1];
        new_pt.z = ptr[2];
        cloud->push_back(new_pt);
    }

    PointCloudDisplay display("line fit cloud");
    display.startSpin();
    display.addPointcloud(cloud, 0);

    pcl::PointXYZ pt1, pt2;
    Eigen::Map<wave::Vec3f> m1(pt1.data), m2(pt2.data);

    m1 =
            (this->global_line.block<3, 1>(3, 0) - 5 * this->global_line.block<3, 1>(0, 0)).cast<float>();
    m2 =
            (this->global_line.block<3, 1>(3, 0) + 5.0 * this->global_line.block<3, 1>(0, 0)).cast<float>();

    display.addLine(pt1, pt2, 1, 2);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    ceres::TinySolver<RobustLineFit> solver;
    auto summary = solver.Solve(line_fit, &(this->local_delta));

    m1 =
            (this->global_line.block<3, 1>(3, 0) - 5 * this->global_line.block<3, 1>(0, 0)).cast<float>();
    m2 =
            (this->global_line.block<3, 1>(3, 0) + 5.0 * this->global_line.block<3, 1>(0, 0)).cast<float>();

    display.addLine(pt1, pt2, 3, 4);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    EXPECT_TRUE(summary.initial_cost > summary.final_cost);

    display.stopSpin();
}

}