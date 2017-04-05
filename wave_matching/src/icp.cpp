#include "wave/utils/config.hpp"
#include "wave/matching/icp.hpp"

namespace wave {

ICPMatcher::ICPMatcher(float res) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }
    wave::ConfigParser parser;

    double max_corr = 3, t_eps = 1e-8, fit_eps = 1e-2;
    int max_iter = 100;

    parser.addParam("icp.maxCorrespondence", &max_corr);
    parser.addParam("icp.maxIterations", &max_iter);
    parser.addParam("icp.transformationEpsilon", &t_eps);
    parser.addParam("icp.euclideanFitnessEpsilon", &fit_eps);

    parser.load("config/icp.yaml");

    this->icp.setMaxCorrespondenceDistance(max_corr);
    this->icp.setMaximumIterations(max_iter);
    this->icp.setTransformationEpsilon(t_eps);
    this->icp.setEuclideanFitnessEpsilon(fit_eps);
}

void ICPMatcher::setRef(const PCLPointCloud &ref) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->icp.setInputSource(this->ref);
}

void ICPMatcher::setTarget(const PCLPointCloud &target) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->icp.setInputTarget(this->target);
}

bool ICPMatcher::match() {
    this->icp.align(*(this->final));
    if (this->icp.hasConverged()) {
        this->result.matrix() = icp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // end of wave namespace