#include <benchmark/benchmark.h>
#include <Eigen/Eigen>
#include <nabo/nabo.h>

#include "nanoflanntesttype.hpp"

static void LIBNABO_NN_BUILD(benchmark::State &state) {
    Eigen::MatrixXf dataset = Eigen::MatrixXf::Random(3, state.range(0));
    Nabo::NNSearchF* nns;

    for (auto _ : state) {
       nns = Nabo::NNSearchF::createKDTreeLinearHeap(dataset);
    }
    delete nns;
}
BENCHMARK(LIBNABO_NN_BUILD)->RangeMultiplier(10)->Range(10, 10000);

static void NANOFLANN_NN_BUILD(benchmark::State &state) {
    wave::FeatureKDTree<float> data;
    data.points = Eigen::MatrixXf::Random(3, state.range(0));

    wave::kd_tree_t<float> kdtree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(8));

    for (auto _ : state) {
        kdtree.buildIndex();
    }
}
BENCHMARK(NANOFLANN_NN_BUILD)->RangeMultiplier(10)->Range(10, 10000);

static void LIBNABO_EXACT_NN_SEARCH(benchmark::State &state) {
    Eigen::MatrixXf dataset = Eigen::MatrixXf::Random(3, 10000);
    Eigen::MatrixXf query = Eigen::MatrixXf::Random(3, state.range(0));

    Nabo::NNSearchF* nns = Nabo::NNSearchF::createKDTreeLinearHeap(dataset);

    Eigen::MatrixXi indices;
    Eigen::MatrixXf dists2;

    // Look for the range nearest neighbours of each query point,
    // We do not want approximations but we want to sort by the distance,
    indices.resize(state.range(1), query.cols());
    dists2.resize(state.range(1), query.cols());
    for (auto _ : state) {
        nns->knn(query, indices, dists2, state.range(1), 0, Nabo::NNSearchF::SORT_RESULTS);
    }
    delete nns;
}
BENCHMARK(LIBNABO_EXACT_NN_SEARCH)->RangeMultiplier(10)->Ranges({{10, 10000}, {1, 10}});

/// nanoflann seems to crap out higher than 10000 point

static void NANOFLANN_EXACT_NN_SEARCH(benchmark::State &state) {
    wave::FeatureKDTree<float> data;
    data.points = Eigen::MatrixXf::Random(3, 10000);
    Eigen::MatrixXf query = Eigen::MatrixXf::Random(3, state.range(0));

    wave::kd_tree_t<float> kdtree(3, data, nanoflann::KDTreeSingleIndexAdaptorParams(8));

    const size_t num_results = state.range(1);
    std::vector<size_t>   ret_indexes(num_results);
    std::vector<float> out_dists_sqr(num_results);

    nanoflann::KNNResultSet<float> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

    kdtree.buildIndex();
    for (auto _ : state) {
        auto ptr = query.data();
        for (long i = 0; i < state.range(0); i++) {
            kdtree.knnSearch(ptr, state.range(1), &ret_indexes[0], &out_dists_sqr[0]);
            ptr = ptr + 3;
        }
    }
}
BENCHMARK(NANOFLANN_EXACT_NN_SEARCH)->RangeMultiplier(10)->Ranges({{10, 10000}, {1, 10}});

static void LIBNABO_APPROX_NN_SEARCH(benchmark::State &state) {
    Eigen::MatrixXf dataset = Eigen::MatrixXf::Random(3, 6400);
    Eigen::MatrixXf query = Eigen::MatrixXf::Random(3, 6400);

    Nabo::NNSearchF* nns = Nabo::NNSearchF::createKDTreeLinearHeap(dataset);

    Eigen::MatrixXi indices;
    Eigen::MatrixXf dists2;

    // Look for the 10 nearest neighbours
    indices.resize(10, query.cols());
    dists2.resize(10, query.cols());

    float approx = 0.01f * static_cast<float>(state.range(0));

    for (auto _ : state) {
        nns->knn(query, indices, dists2, 10, approx, Nabo::NNSearchF::SORT_RESULTS);
    }
    delete nns;
}
BENCHMARK(LIBNABO_APPROX_NN_SEARCH)->RangeMultiplier(2)->Range(1, 16);

BENCHMARK_MAIN();