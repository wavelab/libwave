#include "wave/utils/config.hpp"
#include "wave/matching/g_icp.hpp"

namespace wave {

GICPMatcher::GICPMatcher(float res) {
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

    double r_eps = 1e-8, fit_eps = 1e-2;
    int corr_rand = 10, max_iter = 100;

    parser.addParam("g_icp.correspondenceRandomness", &corr_rand);
    parser.addParam("g_icp.maxIterations", &max_iter);
    parser.addParam("g_icp.rotationEpsilon", &r_eps);
    parser.addParam("g_icp.euclideanFitnessEpsilon", &fit_eps);

    parser.load("config/g_icp.yaml");

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