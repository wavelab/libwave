#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <iostream>
#include <unordered_map>
#include "wave/containers/measurement_container.hpp"

namespace wave {

/** A simple measurement type used for this benchmark */
struct TestMeas {
    int time_point;
    int sensor_id;
    double value;

    TestMeas() = default;
    TestMeas(int t, int s, double v) : time_point{t}, sensor_id{s}, value{v} {}
};

/** The corresponding interpolate function, required by MeasurementContainer */
double interpolate(const TestMeas &m1, const TestMeas &m2, const double &t) {
    auto w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);
    return (1 - w2) * m1.value + w2 * m2.value;
}

/** Returns a random double. */
double random(double start = 0, double end = 1) {
    // To avoid doing any work use Eigen's implementation
    return Eigen::internal::random<double>(start, end);
}

/** A bare bones boost::multi_index_container type, with only one index. It is
 * here to serve as a baseline against which to compare  MeasurementContainer.
 * It was also helpful as a sanity check while writing the benchmarks
 * themselves, since the complexity of its operations is known.
 */
using BaselineMIC = boost::multi_index_container<
  TestMeas,
  boost::multi_index::indexed_by<boost::multi_index::ordered_unique<
    boost::multi_index::member<TestMeas, int, &TestMeas::time_point>>>,
  std::allocator<TestMeas>>;

/** Makes a container with `n` sequential TestMeas elements */
template <typename T>
T makeContainer(int n) {
    auto container = T{};
    for (int i = 0; i < n; ++i) {
        container.emplace(i, 0, random());
    }
    return container;
}

/** Test emplacing an element at the end of the container */
template <typename T>
void BM_ContainerEmplace(benchmark::State &state) {
    const auto size = state.range(0);

    // Prepare a container of the size given by BM
    auto container = makeContainer<T>(size);
    for (auto _ : state) {
        state.PauseTiming();
        // Restore the container to the original size
        for (auto it = container.end();
             container.size() > static_cast<std::size_t>(size);) {
            container.erase(--it);
        }
        auto t = size + 1;
        state.ResumeTiming();

        // Perform actual timed operations
        for (int j = 0; j < state.range(1); ++j) {
            container.emplace(t++, 0, random());
        }
    }

    // Use this benchmark to caculate big O complexity
    state.SetComplexityN(size);
}

/** Test finding an element in the baseline container */
void BM_BaselineGet(benchmark::State &state) {
    const auto size = state.range(0);

    // Prepare a container of the size given by BM
    auto container = makeContainer<BaselineMIC>(size);

    for (auto _ : state) {
        // Request an element with a random time_point
        auto t = static_cast<int>(std::floor(random(0, size)));
        const auto res = container.find(t);
        benchmark::DoNotOptimize(res);
    }

    // Use this benchmark to caculate big O complexity
    state.SetComplexityN(size);
}

/** Test finding an element in the MeasurementContainer
 *
 * Note this is not a templated benchmark due to API differences between the
 * baseline MIC and MeasurementContainer.
 */
void BM_ContainerGet(benchmark::State &state) {
    const auto size = state.range(0);

    // Prepare a container of the size given by BM
    auto container = makeContainer<MeasurementContainer<TestMeas>>(size);

    for (auto _ : state) {
        // Request an element with a random time_point
        auto t = static_cast<int>(std::floor(random(0, size)));
        auto res = container.get(t, 0);
        benchmark::DoNotOptimize(res);
    }

    // Use this benchmark to caculate big O complexity
    state.SetComplexityN(size);
}

// Configure the benchmarks to run

BENCHMARK_TEMPLATE(BM_ContainerEmplace, BaselineMIC)
  ->Ranges({{1 << 15, 1 << 20}, {1, 10}})
  ->Complexity();

BENCHMARK_TEMPLATE(BM_ContainerEmplace, MeasurementContainer<TestMeas>)
  ->Ranges({{1 << 15, 1 << 20}, {1, 10}})
  ->Complexity();

BENCHMARK(BM_BaselineGet)->Range(1 << 12, 1 << 22)->Complexity();

BENCHMARK(BM_ContainerGet)->Range(1 << 8, 1 << 18)->Complexity();

}  // namespace wave

BENCHMARK_MAIN();
