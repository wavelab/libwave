#include "wave/matching/ICP.hpp"

namespace wave {
namespace matching {

ICP_Matcher::ICP_Matcher(float res) {
    resolution = res;
    filter.setLeafSize(res, res, res);

    // Set the max correspondence distance
    icp.setMaxCorrespondenceDistance (1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
}

void ICP_Matcher::setRef(const PCLPointCloud &ref) {
    if(resolution > 0) {
        filter.setInputCloud(ref);
        filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->icp.setInputSource(this->ref);
}

void ICP_Matcher::setTarget(const PCLPointCloud &target) {
    if(resolution > 0) {
        filter.setInputCloud(target);
        filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->icp.setInputSource(this->target);
}

bool ICP_Matcher::match() {
    icp.align(*(this->final));
    this->result.matrix() = icp.getFinalTransformation().cast<double>();
}
}
}