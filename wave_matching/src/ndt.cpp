#include "wave/utils/config.hpp"
#include "wave/utils/logging.hpp"
#include "wave/matching/ndt.hpp"

namespace wave {

NDTMatcher::NDTMatcher(float res, const std::string &config_path) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    ConfigParser parser;
    float step_size, t_eps, default_res;
    int max_iter;
    parser.addParam("ndt.step_size", &step_size);
    parser.addParam("ndt.max_iter", &max_iter);
    parser.addParam("ndt.t_eps", &t_eps);
    parser.addParam("ndt.default_res", &default_res);

    if (parser.load(config_path) != 0) {
        ConfigException config_exception;
        throw config_exception;
    }

    if (res < this->min_res) {
        if (default_res < this->min_res) {
            LOG_ERROR("Invalid resolution given, using minimum");
            default_res = this->min_res;
        } else {
            this->resolution = default_res;
        }
    } else {
        this->resolution = res;
    }

    this->ndt.setTransformationEpsilon(t_eps);
    this->ndt.setStepSize(step_size);
    this->ndt.setResolution(this->resolution);
    this->ndt.setMaximumIterations(max_iter);
}

NDTMatcher::~NDTMatcher() {
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

void NDTMatcher::setRef(const PCLPointCloud &ref) {
    this->ref = ref;
    this->ndt.setInputSource(this->ref);
}

void NDTMatcher::setTarget(const PCLPointCloud &target) {
    this->target = target;
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
