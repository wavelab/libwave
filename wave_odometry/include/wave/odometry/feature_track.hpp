#ifndef WAVE_FEATURE_TRACK_HPP
#define WAVE_FEATURE_TRACK_HPP

#include <vector>
#include "wave/utils/math.hpp"

namespace wave {

template <int state_dim>
struct FeatureTrack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // indicies to each point within each scan
    struct Mapping {
        uint32_t pt_idx, scan_idx;
    };
    std::vector<Mapping> mapping;
    uint32_t featT_idx;

    // Index to the average of the points
    uint32_t ave_pt_idx;
    // line/plane definition
    Vec6 geometry;

    // state ids, indexed by pointid, then by state number
    std::vector<std::vector<uint32_t>> state_ids;
    uint32_t length;

    // state jacobians. indexed by point, then by state.
    std::vector<std::vector<Eigen::Matrix<double, 3, state_dim>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, state_dim>>>>
      jacs;
};
}

#endif  // WAVE_FEATURE_TRACK_HPP
