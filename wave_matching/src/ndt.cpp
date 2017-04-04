#include "wave/utils/config.hpp"
#include "wave/matching/ndt.hpp"

namespace wave {

NDTMatcher::NDTMatcher(float res, const std::string &config_path) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    ConfigParser parser;

    float step_size = 0.1, t_eps = 0.01, config_res = 3;
    int max_iter = 100;

    parser.addParam("ndt.step_size", &step_size);
    parser.addParam("ndt.max_iter", &max_iter);
    parser.addParam("ndt.t_eps", &t_eps);
    parser.addParam("ndt.config_res", &config_res);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }

    if (res > 0) {  // override resolution in yaml config
        config_res = res;
    }
    this->resolution = config_res;

    this->ndt.setTransformationEpsilon(t_eps);
    this->ndt.setStepSize(step_size);
    this->ndt.setResolution(config_res);
    this->ndt.setMaximumIterations(max_iter);
}

void NDTMatcher::setRef(const PCLPointCloud &ref) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->ndt.setInputSource(this->ref);
}

void NDTMatcher::setTarget(const PCLPointCloud &target) {
    if (resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->ndt.setInputTarget(this->target);
}

bool NDTMatcher::match() {
    this->ndt.align(*(this->final));
    if (this->ndt.hasConverged()) {
        this->result.matrix() = ndt.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // end of namespace wave