#include "wave/utils/config.hpp"
#include "wave/matching/icp.hpp"

namespace wave {

ICPMatcher::ICPMatcher(float res, const std::string &config_path) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }

    ConfigParser parser;
    double max_corr, t_eps, fit_eps;
    int max_iter;
    parser.addParam("icp.max_corr", &max_corr);
    parser.addParam("icp.max_iter", &max_iter);
    parser.addParam("icp.t_eps", &t_eps);
    parser.addParam("icp.fit_eps", &fit_eps);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }

    this->icp.setMaxCorrespondenceDistance(max_corr);
    this->icp.setMaximumIterations(max_iter);
    this->icp.setTransformationEpsilon(t_eps);
    this->icp.setEuclideanFitnessEpsilon(fit_eps);
}

ICPMatcher::~ICPMatcher() {
    if (this->ref) {
        this->ref.reset();
    }
    if (this->target) {
        this->target.reset();
    }
    if (this->final) {
        this->final.reset();
    }
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

void ICPMatcher::estimate_info() {
    switch (this->estimate_method) {
        case ICPMatcher::covar_method::LUM:
            this->estimate_lum();
        case ICPMatcher::covar_method::CENSI:
            this->estimate_censi();
        default:
            return;
    }
}

void ICPMatcher::estimate_lum() {
    if (this->icp.hasConverged()) {
    
    }
}

void ICPMatcher::estimate_censi() {
    if (this->icp.hasConverged()) {

    }
}

}  // end of wave namespace
