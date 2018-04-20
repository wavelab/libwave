#include <string>
#include <iostream>
#include <pointmatcher/PointMatcher.h>

#include "wave/matching/lpm_icp.hpp"

namespace wave {

LPMMatcher::LPMMatcher(std::string yaml_file) {
    std::ifstream ifs(yaml_file);
    // TODO(jskh): check validity of file
    this->icp.loadFromYaml(ifs);
}

// TODO(jskhu): Create deconstructor
LPMMatcher::~LPMMatcher() {

}

void LPMMatcher::setRef(const boost::shared_ptr<PointMatcher<double>::DataPoints> &ref) {
    this->ref = ref;
}

void LPMMatcher::setTarget(const boost::shared_ptr<PointMatcher<double>::DataPoints> &target) {
    this->target = target;
}

bool LPMMatcher::match() {
    // TODO(jskhu): add more functionality within the LPM for
    // finer results such as multiscale steps
    try {

        this->transform_params = icp(*(this->ref), *(this->target));

        // TODO(jskhu): Add checker to see validity of matching
        this->result.matrix() = this->transform_params.matrix().matrix();
        return true;
    } catch (const PointMatcher<double>::ConvergenceError &e) {
        return false;
    }
}

// TODO(jskhu): Finish estimateInfo (can probably use the same code as OG icp.cpp file in wave_matching
void LPMMatcher::estimateInfo() {
    this->information = this->icp.errorMinimizer->getCovariance().inverse();
}
} // namespace wave