#include "wave/wave_test.hpp"
#include "wave/benchmark/trajectory_compare.hpp"

namespace wave {

TEST(TrajectoryCompare, contructor) {
    TrajectoryCompare compare;
}

TEST(TrajectoryCompare, pushTruth) {
    TrajectoryCompare compare;
    BenchmarkPose testpose;
    testpose.rotation = RotationMd::Identity();
    testpose.translation.setOnes();
    auto now = std::chrono::steady_clock::now();

    compare.pushTruth(testpose, now);
    ASSERT_EQ(1ul, compare.ground_truth.size());
}

TEST(TrajectoryCompare, reset) {
    TrajectoryCompare compare;
    BenchmarkPose testpose;
    testpose.rotation = RotationMd::Identity();
    testpose.translation.setOnes();
    auto now = std::chrono::steady_clock::now();

    compare.pushTruth(testpose, now);
    compare.reset();
    ASSERT_EQ(0ul, compare.ground_truth.size());
}

TEST(TrajectoryCompare, errorStraight) {
    TrajectoryCompare compare;
    BenchmarkPose traj_sample;
    traj_sample.rotation = RotationMd::Identity();
    auto start_t = std::chrono::steady_clock::now();

    for (int i = 0; i < 10; i++) {
        traj_sample.translation = Vec3(i, 0, 0);
        compare.pushTruth(traj_sample, start_t + std::chrono::seconds(i));
        traj_sample.translation = Vec3(i * 1.2, 0, 0);
        compare.pushMeasurement(traj_sample, start_t + std::chrono::seconds(i));
    }
    compare.calculateError();
    EXPECT_EQ(compare.measurements.size(), compare.error.size());

    for (int i = 0; i < 10; i++) {
        EXPECT_NEAR(
          compare.error
            .get(start_t + std::chrono::seconds(i), ComparisonKey::ERROR)
            .translation(0),
          0.2 * i,
          1e-5);
    }
}

TEST(TrajectoryCompare, errorRotation) {
    TrajectoryCompare compare;
    BenchmarkPose truth_sample, traj_sample;
    std::vector<RotationMd> expected_error;

    truth_sample.rotation = RotationMd::Identity();
    truth_sample.translation.setOnes();
    traj_sample.rotation = RotationMd::Identity();
    traj_sample.translation.setOnes();
    auto start_t = std::chrono::steady_clock::now();

    for (int i = 0; i < 10; i++) {
        auto rot = Eigen::AngleAxisd(0.18 * i, Vec3::UnitZ()) *
                   Eigen::AngleAxisd(0.15 * i, Vec3::UnitY()) *
                   Eigen::AngleAxisd(0.2 * i, Vec3::UnitX());
        truth_sample.rotation = RotationMd(rot);
        compare.pushTruth(truth_sample, start_t + std::chrono::seconds(i));
        rot = Eigen::AngleAxisd(0.15 * i, Vec3::UnitZ()) *
              Eigen::AngleAxisd(0.20 * i, Vec3::UnitY()) *
              Eigen::AngleAxisd(0.25 * i, Vec3::UnitX());
        traj_sample.rotation = RotationMd(rot);
        expected_error.push_back(RotationMd(
          truth_sample.rotation.value().transpose() *
          traj_sample.rotation.value()));
        compare.pushMeasurement(traj_sample, start_t + std::chrono::seconds(i));
    }
    compare.calculateError();
    EXPECT_EQ(compare.measurements.size(), compare.error.size());

    for (int i = 0; i < 10; i++) {
        auto time_c = start_t + std::chrono::seconds(i);
        EXPECT_TRUE(expected_error.at(i).isApprox(
          compare.error.get(time_c, ComparisonKey::ERROR).rotation, 1e-5));
    }
}

// This test outputs a file whose format can be checked
TEST(TrajectoryCompare, outputCSV) {
    TrajectoryCompare compare;
    BenchmarkPose truth_sample, traj_sample;

    truth_sample.rotation = RotationMd::Identity();
    truth_sample.translation.setOnes();
    traj_sample.rotation = RotationMd::Identity();
    traj_sample.translation.setOnes();

    auto start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; i++) {
        auto rot = Eigen::AngleAxisd(0.18 * i, Vec3::UnitZ()) *
                   Eigen::AngleAxisd(0.15 * i, Vec3::UnitY()) *
                   Eigen::AngleAxisd(0.2 * i, Vec3::UnitX());
        truth_sample.rotation = RotationMd(rot);
        compare.pushTruth(truth_sample, start_t + std::chrono::seconds(i));
        rot = Eigen::AngleAxisd(0.15 * i, Vec3::UnitZ()) *
              Eigen::AngleAxisd(0.20 * i, Vec3::UnitY()) *
              Eigen::AngleAxisd(0.25 * i, Vec3::UnitX());
        traj_sample.rotation = RotationMd(rot);
        compare.pushMeasurement(traj_sample, start_t + std::chrono::seconds(i));
    }
    compare.calculateError();
    compare.outputCSV("test_output.txt");
}

TEST(RotationInterpolation, quarterTurn) {
    // Setup
    MeasurementContainer<PoseMeasurement> container;
    BenchmarkPose pose(RotationMd(Eigen::Matrix3d::Zero()), Vec3::Zero());

    auto rot = Eigen::AngleAxisd(2, Vec3::UnitX());
    BenchmarkPose pose_rot(RotationMd(rot), Vec3::Zero());
    rot = Eigen::AngleAxisd(0.5, Vec3::UnitX());
    BenchmarkPose expected(RotationMd(rot), Vec3::Zero());
    auto start_t = std::chrono::steady_clock::now();

    container.emplace(start_t, ComparisonKey::GROUND_TRUTH, pose);
    container.emplace(
      start_t + std::chrono::seconds(4), ComparisonKey::GROUND_TRUTH, pose_rot);

    // Test
    BenchmarkPose inter = container.get(start_t + std::chrono::seconds(1),
                                        ComparisonKey::GROUND_TRUTH);
    EXPECT_TRUE(expected.rotation.isApprox(inter.rotation, 0.1));
}

}  // end of namespace wave
