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
#include "wave/odometry/odometry_types.hpp"

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
    float variance_limit_rng = 1;
    float variance_limit_int = 1;

    unlong angular_bins = 12;
    float min_intensity = 10.0;
    float max_intensity = 50.0;
    float occlusion_tol = 0.1;   // Radians
    float occlusion_tol_2 = 1;   // m. Distance between points to initiate occlusion check
    float parallel_tol = 0.002;  // ditto
    float edge_tol = 0.1;       // Edge features must have score higher than this
    float flat_tol = 0.1;       // Plane features must have score lower than this
    float int_edge_tol = 2;     // Intensity edge features must have score greater than this
    float int_flat_tol = 0.1;   // Intensity edges must have range score lower than this
    int n_edge = 40;             // How many edge features to pick out per ring
    int n_flat = 100;            // How many plane features to pick out per ring
    int n_int_edge = 0;          // How many intensity edges to pick out per ring
    unlong knn = 5;              // 1/2 nearest neighbours for computing curvature
    unlong key_radius = 5;       // minimum number of points between keypoints on the same laser ring

    int eigen_threads = 4;

    uint32_t N_SCORES = 5;
    uint32_t N_FEATURES = 5;

    /// Building these definitions is the responsibility of the application
    std::vector<FeatureDefinition> feature_definitions;
};

class FeatureExtractor {
 public:
    // Aliases are handy
    template<typename datatype>
    using Vec = std::vector<datatype>;

    template<typename datatype, typename alloctype>
    using VecA = std::vector<datatype, alloctype>;

    using Tensor2f = Eigen::Tensor<float, 2>;
    // type for incoming signals. Vector is ringsize long, each element is channels x points tensor
    using Tensorf = VecA<Tensor2f, Eigen::aligned_allocator<Tensor2f>>;
    // type for outgoing keypoint indices
    // Each ring and feature type combination has a variable number of feature points, so the container
    // must support both. Indexed by feature id, then ring in that order.
    using TensorIdx = Vec<VecA<Eigen::Tensor<int, 1>, Eigen::aligned_allocator<Eigen::Tensor<int, 1>>>>;

    FeatureExtractor() = default;
    FeatureExtractor(FeatureExtractorParams params, unlong n_rings);

    void setParams(FeatureExtractorParams params, unlong n_rings);

    // Designed as function for outside world to call
    void getFeatures(const Tensorf &scan, const Tensorf &signals,
                     const std::vector<int> &range, TensorIdx &indices);

    FeatureExtractorParams param;

    bool ready = false;
    void setup();

    void computeScores(const Tensorf &signals, const Vec<int> &range);
    void preFilter(const Tensorf &scan, const Tensorf &signals, const Vec<int> &range);
    void buildFilteredScore(const Vec<int> &range);
    void sortAndBin(const Tensorf &scan, TensorIdx &feature_indices);

    void flagNearbyPoints(const uint32_t p_idx, Eigen::Tensor<bool, 1> &valid);

    unlong n_ring; // This is separate from params because it depends on hardware

    // The resulting scores for each signal, grouped by kernel and ring
    VecA<Tensor2f, Eigen::aligned_allocator<Tensor2f>> scores;

    // Whether points are still considered candidates. Required to avoid picking neighbouring points
    VecA<Eigen::Tensor<bool, 1>, Eigen::aligned_allocator<Eigen::Tensor<bool, 1>>> valid_pts;

    // Scoring kernels, indexed along 1st dimension
    VecA<Eigen::Tensor<float, 1>, Eigen::aligned_allocator<Eigen::Tensor<float, 1>>> kernels;

    // Container to sort scores with. Each is built depending on feature specification
    Vec<Vec<Vec<std::pair<unlong, float>>>> filtered_scores;

    std::unique_ptr<Eigen::ThreadPool> threadpool;
    std::unique_ptr<Eigen::ThreadPoolDevice> thrddev;
};
}

#endif  // WAVE_FEATURE_EXTRACTOR_HPP
