#include "wave/wave_test.hpp"
#include "wave/benchmark/trajectory_compare.hpp"

namespace wave {

TEST(trajectorycompare, contructor) {
    TrajectoryCompare compare;
}

TEST(trajectorycompare, pushtest) {
    TrajectoryCompare compare;
    Pose testpose;
    testpose.rotation.setIdentity();
    testpose.translation.setOnes();
    auto now = std::chrono::steady_clock::now();

    compare.pushTruth(testpose, now);
    ASSERT_EQ(1ul, compare.ground_truth.size());
}

TEST(trajectorycompare, resettest) {
    TrajectoryCompare compare;
    Pose testpose;
    testpose.rotation.setIdentity();
    testpose.translation.setOnes();
    auto now = std::chrono::steady_clock::now();

    compare.pushTruth(testpose, now);
    compare.reset();
    ASSERT_EQ(0ul, compare.ground_truth.size());
}

TEST(trajectorycompare, error_straight) {
    TrajectoryCompare compare;
    Pose traj_sample;
    traj_sample.rotation.setIdentity();
    auto start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; i++) {
        traj_sample.translation = Vec3(i, 0, 0);
        compare.pushTruth(traj_sample, start_t + std::chrono::seconds(i));
        traj_sample.translation = Vec3(i * 1.2, 0, 0);
        compare.pushMeasurement(traj_sample,
                                 start_t + std::chrono::seconds(i));
    }
    compare.calculateError();
    EXPECT_EQ(compare.measurements.size(), compare.error.size());
    for (int i = 0; i < 10; i++) {
        EXPECT_NEAR(compare.error.get(start_t + std::chrono::seconds(i), 2)
                      .translation(0),
                    0.2 * i,
                    1e-5);
    }
}

TEST(trajectorycompare, error_rotation) {
    TrajectoryCompare compare;
    Pose truth_sample, traj_sample;
    std::vector<Rotation> expected_error;
    truth_sample.rotation.setIdentity();
    truth_sample.translation.setOnes();
    traj_sample.rotation.setIdentity();
    traj_sample.translation.setOnes();
    auto start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; i++) {
        truth_sample.rotation.setFromExpMap(Vec3(0.2 * i, 0.15 * i, 0.18 * i));
        compare.pushTruth(truth_sample, start_t + std::chrono::seconds(i));
        traj_sample.rotation.setFromExpMap(Vec3(0.25 * i, 0.20 * i, 0.15 * i));
        expected_error.push_back(Rotation().setFromMatrix(
          truth_sample.rotation.toRotationMatrix().transpose() *
          traj_sample.rotation.toRotationMatrix()));
        compare.pushMeasurement(traj_sample,
                                 start_t + std::chrono::seconds(i));
    }
    compare.calculateError();

    EXPECT_EQ(compare.measurements.size(), compare.error.size());
    for (int i = 0; i < 10; i++) {
        auto time_c = start_t + std::chrono::seconds(i);
        EXPECT_TRUE(expected_error.at(i).isNear(
          compare.error.get(time_c, 2).rotation, 1e-5));
    }
}

// This test outputs a file whose format can be checked
TEST(trajectorycompare, csv_output_test) {
    TrajectoryCompare compare;
    Pose truth_sample, traj_sample;
    truth_sample.rotation.setIdentity();
    truth_sample.translation.setOnes();
    traj_sample.rotation.setIdentity();
    traj_sample.translation.setOnes();
    auto start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; i++) {
        truth_sample.rotation.setFromExpMap(Vec3(0.2 * i, 0.15 * i, 0.18 * i));
        compare.pushTruth(truth_sample, start_t + std::chrono::seconds(i));
        traj_sample.rotation.setFromExpMap(Vec3(0.25 * i, 0.20 * i, 0.15 * i));
        compare.pushMeasurement(traj_sample,
                                 start_t + std::chrono::seconds(i));
    }
    compare.calculateError();
    compare.outputCSV("test_output.txt");
}

TEST(rotation_interpolation, quarter) {
    // Setup
    MeasurementContainer<PoseMeasurement> container;
    Pose pose = { Rotation(Vec3::Zero()), Vec3::Zero() };
    Pose pose_rot = { Rotation(Vec3(2, 0, 0)), Vec3::Zero() };
    Pose expected = { Rotation(Vec3(0.5, 0, 0)), Vec3::Zero() };
    auto start_t = std::chrono::steady_clock::now();
    container.emplace(start_t, 0, pose);
    container.emplace(start_t + std::chrono::seconds(4), 0, pose_rot);
    // Test
    Pose inter = container.get(start_t + std::chrono::seconds(1), 0);
    EXPECT_TRUE(expected.rotation.isNear(inter.rotation, 0.1));
}

}  // end of namespace wave