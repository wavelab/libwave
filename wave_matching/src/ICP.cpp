#include "wave/matching/ICP.hpp"

namespace wave {
namespace matching {

ICP_Matcher::ICP_Matcher(float res) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }


    // Set the max correspondence distance
    this->icp.setMaxCorrespondenceDistance (3);
    // Set the maximum number of iterations (criterion 1)
    this->icp.setMaximumIterations (100);
    // Set the transformation epsilon (criterion 2)
    this->icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    this->icp.setEuclideanFitnessEpsilon (1e-2);
}

void ICP_Matcher::setRef(const PCLPointCloud &ref) {
    if(this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->icp.setInputSource(this->ref);
}

void ICP_Matcher::setTarget(const PCLPointCloud &target) {
    if(resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->icp.setInputTarget(this->target);
}

bool ICP_Matcher::match() {
    this->icp.align(*(this->final));
    if(this->icp.hasConverged()) {
        this->result.matrix() = icp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}
}
}