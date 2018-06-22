#ifndef WAVE_FEATURE_TRACK_HPP
#define WAVE_FEATURE_TRACK_HPP

#include <vector>
#include "wave/utils/math.hpp"

namespace wave {

template <int state_dim>
struct FeatureTrack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // indicies to each point within each scan
    std::vector<uint32_t> pt_idx, scan_idx;

    // Index to the average of the points
    uint32_t ave_pt_idx;
    // normal for feature track
    Vec3 normal;

    // corresponding states
    std::vector<uint32_t> p_states;
    std::vector<uint32_t> n_states;
    uint32_t length;

    // transform jacobians. prev is the state before, next is the state after
    std::vector<Eigen::Matrix<double, 3, state_dim>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, state_dim>>>
      prev_jac, next_jac;
};
}

#endif  // WAVE_FEATURE_TRACK_HPP
