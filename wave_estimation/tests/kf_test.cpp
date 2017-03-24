#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/utils/utils.hpp"
#include "wave/estimation/kf.hpp"

#define TEST_KF_OUTPUT_FILE "/tmp/estimation_kf_test.output"


// TESTS
int prepareOutputFile(std::ofstream &output_file, std::string output_path);
void recordTimeStep(std::ofstream &output_file,
                    int i,
                    wave::Vec3 mea,
                    wave::Vec3 est);


int prepareOutputFile(std::ofstream &output_file, std::string output_path) {
    output_file.open(output_path);

    // clang-format off
    output_file << "time_step" << ",";
    output_file << "x" << ",";
    output_file << "y" << ",";
    output_file << "z" << ",";
    output_file << "bx" << ",";
    output_file << "by" << ",";
    output_file << "bz" << std::endl;
    // clang-format on

    return 0;
}

void recordTimeStep(std::ofstream &output_file,
                    int i,
                    wave::Vec3 mea,
                    wave::Vec3 est) {
    // record true state x, y, z
    output_file << i << ",";
    output_file << mea(0) << ",";
    output_file << mea(1) << ",";
    output_file << mea(2) << ",";

    // record belief state x, y, z
    output_file << est(0) << ",";
    output_file << est(1) << ",";
    output_file << est(2) << std::endl;
}

TEST(KalmanFilter, estimate) {
    float dt;

    wave::KalmanFilter kf;
    wave::Vec3 pos;
    wave::Vec3 vel;
    wave::Vec3 acc;
    wave::VecX state(9);
    wave::VecX mu(9);
    wave::MatX A(9, 9);
    wave::MatX R(9, 9);
    wave::MatX C(3, 9);
    wave::MatX Q(3, 3);
    wave::VecX y(3);
    wave::VecX motion_noise(3);
    wave::Vec3 mea;
    wave::Vec3 est;
    std::ofstream output_file;
    std::default_random_engine rgen;
    std::normal_distribution<float> norm_x(0, 0.5);
    std::normal_distribution<float> norm_y(0, 0.5);
    std::normal_distribution<float> norm_z(0, 0.5);

    // setup
    // clang-format off
    dt = 0.1;
    pos << 0, 0, 0;
    vel << 9, 30, 0;
    acc << 0, -10, 0;
    mu << 0.0, 0.0, 0.0,  // x, y, z
      9.0, 30.0, 0.0,     // x_dot, y_dot, z_dot
      0.0, -10.0, 0.0;    // x_ddot, y_ddot, z_ddot
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0.5, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0.5, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1.0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1.0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1.0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1.0;
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;
    Q << 20, 0, 0, 0, 20, 0, 0, 0, 20;
    // clang-format on
    kf.init(mu, R, C, Q);
    prepareOutputFile(output_file, TEST_KF_OUTPUT_FILE);

    // estimate
    for (int i = 0; i < 20; i++) {
        // update true state
        // clang-format off
        vel = vel + acc * dt;
        pos = pos + vel * dt;
        state << pos(0), pos(1), pos(2),
                vel(0), vel(1), vel(2),
                acc(0), acc(1), acc(2);
        // clang-format on

        // perform measurement
        motion_noise << norm_x(rgen), norm_y(rgen), norm_z(rgen);
        y = kf.C * state + motion_noise;

        // estimate
        // clang-format off
        A << 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0, 0,
            0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0,
            0, 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0,
            0, 0, 0, 1.0, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1.0, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1.0, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1.0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1.0;
        // clang-format on
        kf.estimate(A, y);

        // record
        mea << pos(0), pos(1), pos(2);
        est << kf.mu(0), kf.mu(1), kf.mu(2);
        recordTimeStep(output_file, i, mea, est);
    }
    output_file.close();
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
