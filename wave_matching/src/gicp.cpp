#include "wave/utils/config.hpp"
#include "wave/matching/gicp.hpp"

namespace wave {

GICPMatcher::GICPMatcher(float res, const std::string &config_path) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }

    ConfigParser parser;
    double r_eps = 1e-8, fit_eps = 1e-2;
    int corr_rand = 10, max_iter = 100;
    parser.addParam("corr_rand", &corr_rand);
    parser.addParam("max_iter", &max_iter);
    parser.addParam("r_eps", &r_eps);
    parser.addParam("fit_eps", &fit_eps);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }

    this->gicp.setCorrespondenceRandomness(corr_rand);
    this->gicp.setMaximumIterations(max_iter);
    this->gicp.setRotationEpsilon(r_eps);
    this->gicp.setEuclideanFitnessEpsilon(fit_eps);
}

void GICPMatcher::setRef(const PCLPointCloud &ref) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->gicp.setInputSource(this->ref);
}

void GICPMatcher::setTarget(const PCLPointCloud &target) {
    if (resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->gicp.setInputTarget(this->target);
}

bool GICPMatcher::match() {
    this->gicp.align(*(this->final));
    if (this->gicp.hasConverged()) {
        this->result.matrix() = gicp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // end of wave namespace
