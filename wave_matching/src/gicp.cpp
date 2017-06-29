#include "wave/utils/config.hpp"
#include "wave/matching/gicp.hpp"

namespace wave {

GICPMatcherParams::GICPMatcherParams(const std::string &config_path) {
    ConfigParser parser;
    parser.addParam("corr_rand", &this->corr_rand);
    parser.addParam("max_iter", &this->max_iter);
    parser.addParam("r_eps", &this->r_eps);
    parser.addParam("fit_eps", &this->fit_eps);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }
}

GICPMatcher::GICPMatcher(GICPMatcherParams params1) : params(params1) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    if (params.res > 0) {
        this->resolution = params.res;
        this->filter.setLeafSize(params.res, params.res, params.res);
    } else {
        this->resolution = -1;
    }
    this->gicp.setCorrespondenceRandomness(this->params.corr_rand);
    this->gicp.setMaximumIterations(this->params.max_iter);
    this->gicp.setRotationEpsilon(this->params.r_eps);
    this->gicp.setEuclideanFitnessEpsilon(this->params.fit_eps);
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

}  // namespace wave
