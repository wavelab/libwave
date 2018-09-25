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
        Mapping() = default;
        Mapping(uint32_t pt_idx, uint32_t scan_idx) : pt_idx(pt_idx), scan_idx(scan_idx) {}
        uint32_t pt_idx, scan_idx, state_id;
    };
    /// Book-keeping for feature points that have static data association
    std::vector<Mapping> static_mapping;
    /// Book-keeping for feature points whose data association may change between iteration.
    std::vector<Mapping> fluid_mapping;

    // line/plane definition
    Vec6 geometry;

    uint32_t length;
    uint32_t age = 0;
};
}

#endif  // WAVE_FEATURE_TRACK_HPP
