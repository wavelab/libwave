#ifndef WAVE_FEATURE_EXTRACTOR_HPP
#define WAVE_FEATURE_EXTRACTOR_HPP

#ifndef EIGEN_USE_THREADS
#define EIGEN_USE_THREADS
#endif

#include <vector>
#include <array>
#include <algorithm>
#include <exception>

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#include "wave/utils/math.hpp"
#include "wave/utils/data.hpp"
#include "wave/odometry/kernels.hpp"

namespace wave {

using unlong = unsigned long;

template <int size>
using Earr = Eigen::array<long int, size>;
using ar1 = Earr<1>;
using ar2 = Earr<2>;
using ar3 = Earr<3>;

struct FeatureExtractorParams {
    // Feature extraction parameters
    unlong variance_window = 11;
    bool limit_rng_var = false;
    bool limit_int_var = false;
    double variance_limit_rng = 1;
    double variance_limit_int = 1;

    unlong angular_bins = 12;
    float min_intensity = 10.0;
    float max_intensity = 50.0;
    float occlusion_tol = 0.1;   // Radians
    float occlusion_tol_2 = 1;   // m. Distance between points to initiate occlusion check
    float parallel_tol = 0.002;  // ditto
    double edge_tol = 0.1;       // Edge features must have score higher than this
    double flat_tol = 0.1;       // Plane features must have score lower than this
    double int_edge_tol = 2;     // Intensity edge features must have score greater than this
    double int_flat_tol = 0.1;   // Intensity edges must have range score lower than this
    int n_edge = 40;             // How many edge features to pick out per ring
    int n_flat = 100;            // How many plane features to pick out per ring
    int n_int_edge = 0;          // How many intensity edges to pick out per ring
    unlong knn = 5;              // 1/2 nearest neighbours for computing curvature
    unlong key_radius = 5;       // minimum number of points between keypoints on the same laser ring

    int eigen_threads = 4;
};

class FeatureExtractor {
 private:
    template<typename datatype>
    using Vec<datatype> = std::vector<datatype>;

    template<typename datatype, typename alloctype>
    using Vec<datatype, alloctype> = std::vector<datatype, alloctype>;

    using Tensor2f = Eigen::Tensor<float, 2>;
    // type for incoming signals. Vector is ringsize long, each element is channels x points tensor
    using Tensorf = Vec<Tensor2f, Eigen::aligned_allocator<Tensor2f>>;
    // type for outgoing keypoint indices
    // Each ring and feature type combination has a variable number of feature points, so the container
    // must support both
    using TensorIdx = Vec<Vec<Eigen::Tensor<int, 1>, Eigen::aligned_allocator<Eigen::Tensor<int, 1>>>>;

 public:
    FeatureExtractor FeatureExtractor() = default;
    FeatureExtractor FeatureExtractor(FeatureExtractorParams params) : param(param) {}

    void init(unlong n_rings);
    void setParams(FeatureExtractorParams params);

    // Designed as function for outside world to call
    void getFeatures(const Tensorf &scan, const FeatureExtractor::Tensorf &signals,
                     const std::vector<int> &range, FeatureExtractor::TensorIdx &indices);

 private:

    void computeScores(const Tensorf &signals, const Vec<int> &range);
    void preFilter(const Tensorf &scan, const Tensorf &signals, const Vec<int> &range);
    void buildFilteredScore(const Vec<int> &range);
    void sortAndBin();

    enum SelectionPolicy { HIGH_POS, HIGH_NEG, NEAR_ZERO };
    enum Kernel { LOAM, LOG, FOG, RNG_VAR, INT_VAR };
    struct Criteria {
        Kernel kernel;
        SelectionPolicy sel_pol;
        float *threshold;
    };

    struct FeatureDefinition {
        // First item defines sort, rest are logical.
        std::vector<Criteria> criteria;
        int *n_limit;
    };
    std::vector<FeatureDefinition> feature_definitions;

    // This is separate from params because it depends on hardware
    unlong n_ring;
    FeatureExtractorParams param;

    // The resulting scores for each signal, grouped by kernel and ring
    Vec<Tensor2f, Eigen::aligned_allocator<Tensor2f>> scores;

    // Whether points are still considered candidates. Required to avoid picking neighbouring points
    Vec<Eigen::Tensor<bool, 1>, Eigen::aligned_allocator<Eigen::Tensor<bool, 1>>> valid_pts;

    // Scoring kernels, indexed along 1st dimension
    Vec<Eigen::Tensor<float, 1>, Eigen::aligned_allocator<Eigen::Tensor<float, 1>>> kernels;

    // Container to sort scores with. Each is built depending on feature specification
    Vec<Vec<Vec<std::pair<unlong, float>>>> filtered_scores;

    const uint32_t N_SCORES = 5;
    const uint32_t N_FEATURES = 5;

    std::unique_ptr<Eigen::ThreadPool> threadpool;
    std::unique_ptr<Eigen::ThreadPoolDevice> thrddev;
};
}

#endif  // WAVE_FEATURE_EXTRACTOR_HPP
