#include <benchmark/benchmark.h>
#include <pcl/io/pcd_io.h>
#include "wave/utils/config.hpp"
#include "wave/odometry/feature_extractor.hpp"
#include "wave/odometry/PointXYZIR.hpp"

namespace {

void loadParameters(const std::string &path, const std::string &filename, wave::FeatureExtractorParams &params) {
    wave::ConfigParser parser;

    parser.addParam("variance_window", &(params.variance_window));
    parser.addParam("variance_limit_rng", &(params.variance_limit_rng));
    parser.addParam("variance_limit_int", &(params.variance_limit_int));
    parser.addParam("angular_bins", &(params.angular_bins));
    parser.addParam("min_intensity", &(params.min_intensity));
    parser.addParam("max_intensity", &(params.max_intensity));
    parser.addParam("occlusion_tol", &(params.occlusion_tol));
    parser.addParam("occlusion_tol_2", &(params.occlusion_tol_2));
    parser.addParam("parallel_tol", &(params.parallel_tol));
    parser.addParam("edge_tol", &(params.edge_tol));
    parser.addParam("flat_tol", &(params.flat_tol));
    parser.addParam("int_edge_tol", &(params.int_edge_tol));
    parser.addParam("int_flat_tol", &(params.int_flat_tol));
    parser.addParam("n_edge", &(params.n_edge));
    parser.addParam("n_flat", &(params.n_flat));
    parser.addParam("n_int_edge", &(params.n_int_edge));
    parser.addParam("knn", &(params.knn));
    parser.addParam("key_radius", &(params.key_radius));

    parser.load(path + filename);
}

void setupParameters(wave::FeatureExtractorParams &param) {
    std::vector<wave::Criteria> edge_high, edge_low, flat, edge_int_high, edge_int_low;
    edge_high.emplace_back(
      wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_POS, &(param.edge_tol)});

    edge_low.emplace_back(
      wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::HIGH_NEG, &(param.edge_tol)});

    flat.emplace_back(
      wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.flat_tol)});

    edge_int_high.emplace_back(wave::Criteria{
      wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_POS, &(param.int_edge_tol)});
    edge_int_high.emplace_back(
      wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_high.emplace_back(wave::Criteria{
      wave::Signal::RANGE, wave::Kernel::VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    edge_int_low.emplace_back(wave::Criteria{
      wave::Signal::INTENSITY, wave::Kernel::FOG, wave::SelectionPolicy::HIGH_NEG, &(param.int_edge_tol)});
    edge_int_low.emplace_back(
      wave::Criteria{wave::Signal::RANGE, wave::Kernel::LOAM, wave::SelectionPolicy::NEAR_ZERO, &(param.int_flat_tol)});
    edge_int_low.emplace_back(wave::Criteria{
      wave::Signal::RANGE, wave::Kernel::VAR, wave::SelectionPolicy::NEAR_ZERO, &(param.variance_limit_rng)});

    param.feature_definitions.clear();
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_high, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_low, &(param.n_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{flat, &(param.n_flat)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_high, &(param.n_int_edge)});
    param.feature_definitions.emplace_back(wave::FeatureDefinition{edge_int_low, &(param.n_int_edge)});
}

void setup(wave::FeatureExtractorParams &params,
           wave::FeatureExtractor &extractor,
           wave::FeatureExtractor::Tensorf &scan,
           wave::FeatureExtractor::SignalVec &signals,
           std::vector<int> &range,
           wave::FeatureExtractor::TensorIdx &indices) {
    loadParameters("config/", "features.yaml", params);
    uint32_t rings = 32;
    setupParameters(params);
    extractor.setParams(params, rings);

    scan.resize(32);
    signals.resize(32);
    range.resize(32);
    std::fill(range.begin(), range.end(), 0);
    for (uint32_t i = 0; i < 32; i++) {
        scan.at(i) = Eigen::Tensor<float, 2>(5, 2200);
        signals.at(i).emplace(std::make_pair(wave::Signal::RANGE, Eigen::Tensor<float, 1>(2200)));
        signals.at(i).emplace(std::make_pair(wave::Signal::INTENSITY, Eigen::Tensor<float, 1>(2200)));
    }

    indices.resize(params.N_FEATURES);
    for (uint32_t i = 0; i < params.N_FEATURES; i++) {
        indices.at(i).resize(32);
    }

    pcl::PointCloud<wave::PointXYZIR> ref;
    pcl::io::loadPCDFile("data/testscan.pcd", ref);

    for (auto pt : ref) {
        float ang = (float) (std::atan2(pt.y, pt.x) * -1.0);
        // need to fraction of complete rotation
        ang < 0 ? ang = ang + 2.0 * M_PI : ang;

        scan.at(pt.ring)(0, range.at(pt.ring)) = pt.x;
        scan.at(pt.ring)(1, range.at(pt.ring)) = pt.y;
        scan.at(pt.ring)(2, range.at(pt.ring)) = pt.z;
        scan.at(pt.ring)(3, range.at(pt.ring)) = ang / (20 * M_PI);
        scan.at(pt.ring)(4, range.at(pt.ring)) = ang;

        signals.at(pt.ring).at(wave::Signal::RANGE)(range.at(pt.ring)) = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        signals.at(pt.ring).at(wave::Signal::INTENSITY)(range.at(pt.ring)) = pt.intensity;
        range.at(pt.ring) += 1;
    }
}
}

static void BM_FEATURE_EXTRACTION_SCORES(benchmark::State &state) {
    wave::FeatureExtractorParams params;
    wave::FeatureExtractor extractor;
    wave::FeatureExtractor::Tensorf scan;
    wave::FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    wave::FeatureExtractor::TensorIdx indices;

    setup(params, extractor, scan, signals, range, indices);

    for (auto i : state) {
        extractor.computeScores(signals, range);
    }
}
BENCHMARK(BM_FEATURE_EXTRACTION_SCORES);

static void BM_FEATURE_EXTRACTION_PREFILTER(benchmark::State &state) {
    wave::FeatureExtractorParams params;
    wave::FeatureExtractor extractor;
    wave::FeatureExtractor::Tensorf scan;
    wave::FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    wave::FeatureExtractor::TensorIdx indices;

    setup(params, extractor, scan, signals, range, indices);

    extractor.computeScores(signals, range);
    for (auto i : state) {
        extractor.preFilter(scan, signals, range);
    }
}
BENCHMARK(BM_FEATURE_EXTRACTION_PREFILTER);

static void BM_FEATURE_EXTRACTION_FILT_SCORES(benchmark::State &state) {
    wave::FeatureExtractorParams params;
    wave::FeatureExtractor extractor;
    wave::FeatureExtractor::Tensorf scan;
    wave::FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    wave::FeatureExtractor::TensorIdx indices;

    setup(params, extractor, scan, signals, range, indices);

    extractor.computeScores(signals, range);
    extractor.preFilter(scan, signals, range);
    for (auto i : state) {
        extractor.buildFilteredScore(range);
    }
}
BENCHMARK(BM_FEATURE_EXTRACTION_FILT_SCORES);

static void BM_FEATURE_EXTRACTION_SORT(benchmark::State &state) {
    wave::FeatureExtractorParams params;
    wave::FeatureExtractor extractor;
    wave::FeatureExtractor::Tensorf scan;
    wave::FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    wave::FeatureExtractor::TensorIdx indices;

    setup(params, extractor, scan, signals, range, indices);

    extractor.computeScores(signals, range);
    extractor.preFilter(scan, signals, range);
    extractor.buildFilteredScore(range);
    for (auto i : state) {
        extractor.sortAndBin(scan, signals, indices);
    }
}
BENCHMARK(BM_FEATURE_EXTRACTION_SORT);

static void BM_FEATURE_EXTRACTION(benchmark::State &state) {
    wave::FeatureExtractorParams params;
    wave::FeatureExtractor extractor;
    wave::FeatureExtractor::Tensorf scan;
    wave::FeatureExtractor::SignalVec signals;
    std::vector<int> range;
    wave::FeatureExtractor::TensorIdx indices;

    setup(params, extractor, scan, signals, range, indices);

    for (auto i : state) {
        extractor.getFeatures(scan, signals, range, indices);
    }
}

BENCHMARK(BM_FEATURE_EXTRACTION);

// Ensure that StateIterator provides all the necessary typedefs required to
// instantiate std::iterator_traits.
static_assert(std::is_same<typename std::iterator_traits<benchmark::State::StateIterator>::value_type,
                           typename benchmark::State::StateIterator::value_type>::value,
              "");

BENCHMARK_MAIN();
