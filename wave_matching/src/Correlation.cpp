//
// Created by bapskiko on 14/03/17.
//

#include "wave/matching/Correlation.hpp"

namespace wave {
namespace matching {

Correlation_Matcher::Correlation_Matcher(float resolution) :
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
    real_time_correlative_scan_matcher.reset(
            new cartographer::mapping_3d::scan_matching::RealTimeCorrelativeScanMatcher(
                    cartographer::mapping_2d::scan_matching::
                    CreateRealTimeCorrelativeScanMatcherOptions(
                            parameter_dictionary.get())));
}

void Correlation_Matcher::setRef(const CartPointCloud &ref) {
    for (const auto& point : ref) {
        this->hybrid_grid.SetProbability(
                this->hybrid_grid.GetCellIndex(initial_pose.cast<float>() * point), 1.);
    }
}

void Correlation_Matcher::setTarget(const CartPointCloud &target) {
    this->target = target;
}

bool Correlation_Matcher::match() {
    this->real_time_correlative_scan_matcher->Match(this->initial_pose,
                                                    this->target,
                                                    this->hybrid_grid,
                                                    &(this->result_pose));
    this->result = cartographer::transform::ToEigen(this->result_pose);
    return true;
}
}
}
