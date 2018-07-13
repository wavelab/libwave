#ifndef WAVE_FEATURE_TRACK_HPP
#define WAVE_FEATURE_TRACK_HPP

#include <vector>
#include <unsupported/Eigen/CXX11/Tensor>
#include "wave/utils/utils.hpp"

namespace wave {

struct FeatureTrack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // indicies to each point within each scan
    struct Mapping {
        Mapping(uint32_t pt_idx, uint32_t scan_idx) : pt_idx(pt_idx), scan_idx(scan_idx) {}
        uint32_t pt_idx, scan_idx;
    };
    std::vector<Mapping> mapping;

    // line/plane definition
    Vec6 geometry;

    // state ids, indexed by point_idx, then by state number
    std::vector<std::vector<uint32_t>> state_ids;
    uint32_t length;

    /// pointer to state jacobians updated by evaluation callback
    /// dimensions of tensor should be 3 x M x n_pts
    // Indexed by scan ID, then by state_id.
    const Vec<VecE<Eigen::Tensor<double, 3>>> *jacs;
};
}

#endif  // WAVE_FEATURE_TRACK_HPP
