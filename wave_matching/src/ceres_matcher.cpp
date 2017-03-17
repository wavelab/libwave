//
// Created by bapskiko on 14/03/17.
//

#include "wave/matching/ceres_matcher.hpp"

namespace wave {
namespace matching {

DEFINE_string(configuration_directory, "../config", "Where to search for config files");
DEFINE_string(configuration_basename, "ceres_config.lua", "What the config file is named");

CeresMatcher::CeresMatcher(float resolution) :
    hybrid_grid(resolution,
                Eigen::Vector3f(0.0f, 0.0f, 0.0f) /* origin */),
    initial_pose(Eigen::Vector3d(0, 0, 0),
                 Eigen::Quaterniond::Identity()){

    auto file_resolver = cartographer::common::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<string>{FLAGS_configuration_directory});
    const string code =
            file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
    this->dict.reset(new cartographer::common::LuaParameterDictionary(code, std::move(file_resolver)));

    this->ceres_matcher.reset(
            new cartographer::mapping_3d::scan_matching::CeresScanMatcher(
                    cartographer::mapping_3d::scan_matching::
                    CreateCeresScanMatcherOptions(
                            this->dict.get())));
}

void CeresMatcher::setRef(const CartPointCloud &ref) {
    for (const auto& point : ref) {
        this->hybrid_grid.SetProbability(
                this->hybrid_grid.GetCellIndex(initial_pose.cast<float>() * point), 1.);
    }
}

void CeresMatcher::setTarget(const CartPointCloud &target) {
    this->target = target;
}

bool CeresMatcher::match() {
    // result pose is used as an initial estimate, so zero it out before matching
    this->result_pose = cartographer::transform::Rigid3d(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
    // arguments (in order):
    // previous pose: if translation weight is >0, there is a residual block penalizing translation away from
    // this pose
    //
    // initial pose estimate: if rotation weight is >0, a residual block penalizing rotation away from this pose
    //
    // point clouds and hybrid grids: a std::vector with each element being a point cloud and hybrid grid (occupancy
    // grid) to match
    //
    // pose_estimate: Pose estimate to use for solving
    //
    // summary: returned ceres solver summary
    this->ceres_matcher->Match(this->initial_pose, this->initial_pose,
                               {{&(this->target), &(this->hybrid_grid)}}, &(this->result_pose), (&this->summary));
    this->result = cartographer::transform::ToEigen(this->result_pose);
    return true;
}

}  // namespace matching
}  // namespace wave
