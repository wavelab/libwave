#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/utils/utils.hpp"
#include "wave/estimation/ekf.hpp"

#define TEST_EKF_OUTPUT_FILE "/tmp/estimation_ekf_test.output"


// TESTS
int prepareOutputFile(std::ofstream &output_file, std::string output_path);
void recordTimeStep(std::ofstream &output_file,
                    int i,
                    wave::Vec3 mea,
                    wave::Vec3 est);

int prepareOutputFile(std::ofstream &output_file, std::string output_path) {
  output_file.open(output_path);

  output_file << "time_step"
              << ",";
  output_file << "x"
              << ",";
  output_file << "y"
              << ",";
  output_file << "z"
              << ",";
  output_file << "bx"
              << ",";
  output_file << "by"
              << ",";
  output_file << "bz" << std::endl;

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

TEST(ExtendedKalmanFilter, estimate) {
  float dt;
  wave::VecX x(3);
  wave::VecX y(3);
  wave::VecX gaussian_noise(3);
  wave::VecX mu(3);
  wave::VecX u(3);
  wave::MatX R(3, 3);
  wave::MatX Q(3, 3);
  wave::VecX g(3);
  wave::MatX G(3, 3);
  wave::VecX h(3);
  wave::MatX H(3, 3);
  wave::ExtendedKalmanFilter ekf;
  std::ofstream output_file;
  std::default_random_engine rgen;
  std::normal_distribution<float> norm_x(0, pow(0.5, 2));
  std::normal_distribution<float> norm_y(0, pow(0.5, 2));
  std::normal_distribution<float> norm_theta(0, pow(wave::deg2rad(0.5), 2));

  // setup
  // clang-format off
  dt = 0.01;
  x << 0, 0, 0;
  mu << 0, 0, 0;
  R << pow(0.05, 2), 0, 0,
       0, pow(0.05, 2), 0,
       0, 0, pow(wave::deg2rad(0.5), 2);
  Q << pow(0.5, 2), 0, 0,
       0, pow(0.5, 2), 0,
       0, 0, pow(wave::deg2rad(10), 2);
  u << -15.5, -10.5, 1.5;
  // clang-format on
  ekf.init(mu, R, Q);
  prepareOutputFile(output_file, TEST_EKF_OUTPUT_FILE);

  // loop
  for (int i = 0; i < 100; i++) {
    // update true state
    // clang-format off
    x << x(0) + u(0) * cos(x(2)) * dt,
         x(1) + u(0) * sin(x(2)) * dt,
         x(2) + u(1) * dt;
    // clang-format on

    // take measurement
    gaussian_noise << norm_x(rgen), norm_y(rgen), norm_theta(rgen);
    y = x + gaussian_noise;

    // propagate motion model
    // clang-format off
    g << ekf.mu(0) + u(0) * cos(ekf.mu(2)) * dt,
         ekf.mu(1) + u(0) * sin(ekf.mu(2)) * dt,
         ekf.mu(2) + u(1) * dt;
    G << 1, 0, (-u(0) * sin(ekf.mu(2)) * dt),
         0, 1, (u(0) * cos(ekf.mu(2)) * dt),
         0, 0, 1;
    // clang-format on
    ekf.predictionUpdate(g, G);

    // propagate measurement
    H = Eigen::MatrixXf::Identity(3, 3);
    h = H * ekf.mu;
    ekf.measurementUpdate(h, H, y);

    // record
    recordTimeStep(output_file, i, x, ekf.mu);
  }
  output_file.close();
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
