#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include "wave/odometry/geometry/line.hpp"
#include "wave/odometry/geometry/plane.hpp"

static void BM_LINE_ERROR_ONLY(benchmark::State &state) {
    wave::Vec3 pt;
    wave::VecE<wave::MatX> jacsw1, jacsw2;
    wave::VecE<const wave::MatX*> jw1, jw2;
    jacsw1.resize(4);
    jacsw2.resize(4);
    for (uint32_t i = 0; i < jacsw1.size(); ++i) {
        jacsw1.at(i).resize(6,6);
        jacsw2.at(i).resize(6,6);
        jacsw1.at(i).setRandom();
        jacsw2.at(i).setRandom();
        jw1.emplace_back(&(jacsw1.at(i)));
        jw2.emplace_back(&(jacsw2.at(i)));
    }

    double w1, w2;
    w1 = 0.2;
    w2 = 0.8;

    wave::LineResidual<double, 6, 12, 6, 12> residual(pt.data(), jw1, jw2, w1, w2);

    wave::Vec6 line;
    line.setRandom();
    line.block<3,1>(0,0).normalize();

    double residuals[3];
    double *parameters[1];
    parameters[0] = line.data();

    for (auto i : state) {
        residual.Evaluate(parameters, residuals, nullptr);
    }
}
BENCHMARK(BM_LINE_ERROR_ONLY);

static void BM_LINE_WITH_JACOBIAN(benchmark::State &state) {
    wave::Vec3 pt;
    wave::VecE<wave::MatX> jacsw1, jacsw2;
    wave::VecE<const wave::MatX*> jw1, jw2;
    jacsw1.resize(4);
    jacsw2.resize(4);
    for (uint32_t i = 0; i < jacsw1.size(); ++i) {
        jacsw1.at(i).resize(6,6);
        jacsw2.at(i).resize(6,6);
        jacsw1.at(i).setRandom();
        jacsw2.at(i).setRandom();
        jw1.emplace_back(&(jacsw1.at(i)));
        jw2.emplace_back(&(jacsw2.at(i)));
    }

    double w1, w2;
    w1 = 0.2;
    w2 = 0.8;

    wave::LineResidual<double, 6, 12, 6, 12> residual(pt.data(), jw1, jw2, w1, w2);

    wave::Vec6 line;
    line.setRandom();
    line.block<3,1>(0,0).normalize();

    double residuals[3];
    double *parameters[1];
    parameters[0] = line.data();

    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J_line;
    Eigen::Matrix<double, 3, 12, Eigen::RowMajor> J_T1;
    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J_V1;
    Eigen::Matrix<double, 3, 12, Eigen::RowMajor> J_T2;
    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J_V2;

    double *jacobians[5];
    jacobians[0] = J_line.data();
    jacobians[1] = J_T1.data();
    jacobians[2] = J_V1.data();
    jacobians[3] = J_T2.data();
    jacobians[4] = J_V2.data();


    for (auto i : state) {
        residual.Evaluate(parameters, residuals, jacobians);
    }
}
BENCHMARK(BM_LINE_WITH_JACOBIAN);

static void BM_PLANE_ERROR_ONLY(benchmark::State &state) {
    wave::Vec3 pt;
    wave::VecE<wave::MatX> jacsw1, jacsw2;
    wave::VecE<const wave::MatX*> jw1, jw2;
    jacsw1.resize(4);
    jacsw2.resize(4);
    for (uint32_t i = 0; i < jacsw1.size(); ++i) {
        jacsw1.at(i).resize(6,6);
        jacsw2.at(i).resize(6,6);
        jacsw1.at(i).setRandom();
        jacsw2.at(i).setRandom();
        jw1.emplace_back(&(jacsw1.at(i)));
        jw2.emplace_back(&(jacsw2.at(i)));
    }

    double w1, w2;
    w1 = 0.2;
    w2 = 0.8;

    wave::PlaneResidual<double, 6, 12, 6, 12> residual(pt.data(), jw1, jw2, w1, w2);

    wave::Vec6 plane;
    plane.setRandom();
    plane.block<3,1>(0,0).normalize();

    double residuals[1];
    double *parameters[1];
    parameters[0] = plane.data();

    for (auto i : state) {
        residual.Evaluate(parameters, residuals, nullptr);
    }
}
BENCHMARK(BM_PLANE_ERROR_ONLY);

static void BM_PLANE_WITH_JACOBIAN(benchmark::State &state) {
    wave::Vec3 pt;
    wave::VecE<wave::MatX> jacsw1, jacsw2;
    wave::VecE<const wave::MatX*> jw1, jw2;
    jacsw1.resize(4);
    jacsw2.resize(4);
    for (uint32_t i = 0; i < jacsw1.size(); ++i) {
        jacsw1.at(i).resize(6,6);
        jacsw2.at(i).resize(6,6);
        jacsw1.at(i).setRandom();
        jacsw2.at(i).setRandom();
        jw1.emplace_back(&(jacsw1.at(i)));
        jw2.emplace_back(&(jacsw2.at(i)));
    }

    double w1, w2;
    w1 = 0.2;
    w2 = 0.8;

    wave::PlaneResidual<double, 6, 12, 6, 12> residual(pt.data(), jw1, jw2, w1, w2);

    wave::Vec6 plane;
    plane.setRandom();
    plane.block<3,1>(0,0).normalize();

    double residuals[1];
    double *parameters[1];
    parameters[0] = plane.data();

    Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J_line;
    Eigen::Matrix<double, 1, 12, Eigen::RowMajor> J_T1;
    Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J_V1;
    Eigen::Matrix<double, 1, 12, Eigen::RowMajor> J_T2;
    Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J_V2;

    double *jacobians[5];
    jacobians[0] = J_line.data();
    jacobians[1] = J_T1.data();
    jacobians[2] = J_V1.data();
    jacobians[3] = J_T2.data();
    jacobians[4] = J_V2.data();


    for (auto i : state) {
        residual.Evaluate(parameters, residuals, jacobians);
    }
}
BENCHMARK(BM_PLANE_WITH_JACOBIAN);

// Ensure that StateIterator provides all the necessary typedefs required to
// instantiate std::iterator_traits.
static_assert(std::is_same<typename std::iterator_traits<benchmark::State::StateIterator>::value_type,
                           typename benchmark::State::StateIterator::value_type>::value,
              "");

BENCHMARK_MAIN();
