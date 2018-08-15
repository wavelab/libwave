#include <benchmark/benchmark.h>
#include "wave/odometry/transformer.hpp"
#include "wave/matching/pointcloud_display.hpp"

namespace {

void setup(std::vector<wave::PoseVel, Eigen::aligned_allocator<wave::PoseVel>> &trajectory,
           std::vector<float> &stamps, wave::TransformerParams &params,
           Eigen::Tensor<float, 2> &scan, long n_pts) {
    params.n_scans = 4;
    params.traj_resolution = 3;
    params.delRTol = 1e-3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    wave::Vec6 vel;
    vel << 0, 0, 1.5, 3, 0, 2;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(vel * 0.1);
        }
        trajectory.at(i).vel = vel;
    }

    for (long j = 0; j < params.n_scans; j++) {
        for (long i = 0; i < n_pts; i++) {
            scan(0,j*n_pts + i) = static_cast<float>(std::cos(2*M_PI * static_cast<double>(i)/100.0));
            scan(1,j*n_pts + i) = static_cast<float>(std::sin(2*M_PI * static_cast<double>(i)/100.0));
            scan(2,j*n_pts + i) = 0.0f;
            scan(3,j*n_pts + i) = 0.2f * (float)j + 0.2f * (static_cast<float>(i)/static_cast<float>(n_pts + 1));
        }
    }
}

}

static void BM_TRANSFORM_UPDATE(benchmark::State &state) {
    std::vector<wave::PoseVel, Eigen::aligned_allocator<wave::PoseVel>> trajectory;
    std::vector<float> stamps;

    wave::TransformerParams params;
    params.n_scans = 4;
    params.traj_resolution = 3;
    uint32_t cnt = params.n_scans * (params.traj_resolution - 1) + 1;

    wave::Vec6 vel;
    vel << 0, 0, 1.5, 3, 0, 2;

    stamps.resize(cnt);
    trajectory.resize(cnt);

    for(uint32_t i = 0; i < cnt; i++) {
        stamps.at(i) = static_cast<float>(i) * 0.1f;
        if (i != 0) {
            trajectory.at(i).pose = trajectory.at(i - 1).pose;
            trajectory.at(i).pose.manifoldPlus(vel * 0.1);
        }
        trajectory.at(i).vel = vel;
    }
    wave::Transformer transformer(params);

    for (auto _ : state) {
        transformer.update(trajectory, stamps);
    }
}

BENCHMARK(BM_TRANSFORM_UPDATE);

static void BM_TRANSFORM_N(benchmark::State &state) {
    std::vector<wave::PoseVel, Eigen::aligned_allocator<wave::PoseVel>> trajectory;
    std::vector<float> stamps;
    wave::TransformerParams params;
    params.n_scans = 4;
    long n_pts = state.range(0) / params.n_scans;
    Eigen::Tensor<float, 2> scan(4, params.n_scans * n_pts);
    wave::MatXf tscan(3, params.n_scans * n_pts);

    setup(trajectory, stamps, params, scan, n_pts);

    wave::Transformer transformer(params);
    transformer.update(trajectory, stamps);

    for (auto _ : state) {
        transformer.transformToStart(scan, tscan, 0);
    }
}

BENCHMARK(BM_TRANSFORM_N)->RangeMultiplier(10)->Range(1000, 1000000);

// Ensure that StateIterator provides all the necessary typedefs required to
// instantiate std::iterator_traits.
static_assert(std::is_same<typename std::iterator_traits<benchmark::State::StateIterator>::value_type,
                      typename benchmark::State::StateIterator::value_type>::value,
              "");

BENCHMARK_MAIN();