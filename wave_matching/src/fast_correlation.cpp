//
// Created by bapskiko on 14/03/17.
//

#include "wave/matching/fast_correlation.hpp"

namespace wave {
namespace matching {

DEFINE_string(configuration_directory, "../config", "Where to search for config files");
DEFINE_string(configuration_basename, "config.lua", "What the config file is named");

FastCorrelationMatcher::FastCorrelationMatcher(float resolution) :
    hybrid_grid(resolution,
                Eigen::Vector3f(0.0f, 0.0f, 0.0f) /* origin */),
    initial_pose(Eigen::Vector3d(0, 0, 0),
                 Eigen::Quaterniond::Identity())
    {
}

void FastCorrelationMatcher::setRef(const CartPointCloud &ref) {
    auto file_resolver = cartographer::common::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<string>{FLAGS_configuration_directory});
    const string code =
            file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
    this->dict.reset(new cartographer::common::LuaParameterDictionary(code, std::move(file_resolver)));
    cartographer::mapping_3d::LaserFanInserter laser_fan(
            cartographer::mapping_3d::CreateLaserFanInserterOptions(this->dict.get()));
    //this->hybrid_grid.StartUpdate();
    cartographer::sensor::LaserFan fan{this->initial_pose.translation().cast<float>(),
                                         ref,
                                         {}};
    laser_fan.Insert(fan, &(this->hybrid_grid));

    this->fast_correlative_scan_matcher.reset(new cartographer::mapping_3d::scan_matching::FastCorrelativeScanMatcher(
            this->hybrid_grid,
            {},
            cartographer::mapping_3d::scan_matching::CreateFastCorrelativeScanMatcherOptions(this->dict.get())));
}

void FastCorrelationMatcher::setTarget(const CartPointCloud &target) {
    this->target = target;
}

bool FastCorrelationMatcher::match() {
    // result pose is used as an initial estimate, so zero it out before matching
    float score;
    this->result_pose = cartographer::transform::Rigid3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());

    if (this->fast_correlative_scan_matcher->Match(this->initial_pose,
                                                   this->target,
                                                   this->target,
                                                   this->kMinScore,
                                                   &score,
                                                   &(this->result_pose))) {
        // match was successful
        this->result = cartographer::transform::ToEigen(this->result_pose);
        return true;
    }
    return false;
}
}
}
