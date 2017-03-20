#include "wave/matching/NDT.hpp"

namespace wave {
namespace matching {

NDT_Matcher::NDT_Matcher(float res) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }
    this->ndt.setTransformationEpsilon(0.01);
    this->ndt.setStepSize(0.1);
    // This isn't really resolution, this is covariance for the transform
    this->ndt.setResolution(1.0);
    this->ndt.setMaximumIterations(100);
}

void NDT_Matcher::setRef(const PCLPointCloud &ref) {
    if(this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->ndt.setInputSource(this->ref);
}

void NDT_Matcher::setTarget(const PCLPointCloud &target) {
    if(resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->ndt.setInputTarget(this->target);
}

bool NDT_Matcher::match() {
    this->ndt.align(*(this->final));
    if(this->ndt.hasConverged()) {
        this->result.matrix() = ndt.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}
}
}