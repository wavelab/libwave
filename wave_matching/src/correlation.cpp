//
// Created by bapskiko on 14/03/17.
//

#include "wave/matching/correlation.hpp"
#include <cartographer/common/lua_parameter_dictionary_test_helpers.h>

namespace wave {
namespace matching {

CorrelationMatcher::CorrelationMatcher(float resolution) :
    hybrid_grid(resolution,
                Eigen::Vector3f(0.0f, 0.0f, 0.0f) /* origin */),
    initial_pose(Eigen::Vector3d(0, 0, 0),
                 Eigen::Quaterniond::Identity()){
    // set up cartographer matcher
    auto parameter_dictionary = cartographer::common::MakeDictionary(R"text(
        return {
          linear_search_window = 0.5,
          angular_search_window = math.rad(1.),
          translation_delta_cost_weight = 1e-1,
          rotation_delta_cost_weight = 1.,
        })text");
    this->real_time_correlative_scan_matcher.reset(
            new cartographer::mapping_3d::scan_matching::RealTimeCorrelativeScanMatcher(
                    cartographer::mapping_2d::scan_matching::
                    CreateRealTimeCorrelativeScanMatcherOptions(
                            parameter_dictionary.get())));
}

void CorrelationMatcher::setRef(const CartPointCloud &ref) {
    for (const auto& point : ref) {
        this->hybrid_grid.SetProbability(
                this->hybrid_grid.GetCellIndex(initial_pose.cast<float>() * point), 1.);
    }
}

void CorrelationMatcher::setTarget(const CartPointCloud &target) {
    this->target = target;
}

bool CorrelationMatcher::match() {
    // result pose is used as an initial estimate, so zero it out before matching
    this->result_pose = cartographer::transform::Rigid3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
    if(0 < this->real_time_correlative_scan_matcher->Match(this->initial_pose,
                                                    this->target,
                                                    this->hybrid_grid,
                                                    &(this->result_pose))) {
        this->result = cartographer::transform::ToEigen(this->result_pose);
        return true;
    }
    return false;
}

}  // namespace matching
}  // namespace wave
