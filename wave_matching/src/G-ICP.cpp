#include "wave/matching/G-ICP.hpp"

namespace wave {
namespace matching {

GICP_Matcher::GICP_Matcher(float res) {
    this->ref = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    if (res > 0) {
        this->resolution = res;
        this->filter.setLeafSize(res, res, res);
    } else {
        this->resolution = -1;
    }


    // Number of neighbours to use to compute covariance
    this->gicp.setCorrespondenceRandomness(10);
    // Set the maximum number of iterations (criterion 1)
    this->gicp.setMaximumIterations (100);
    // Set the rotation epsilon (criterion 2)
    this->gicp.setRotationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    this->gicp.setEuclideanFitnessEpsilon (1e-2);
}

void GICP_Matcher::setRef(const PCLPointCloud &ref) {
    if(this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->gicp.setInputSource(this->ref);
}

void GICP_Matcher::setTarget(const PCLPointCloud &target) {
    if(resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->gicp.setInputTarget(this->target);
}

bool GICP_Matcher::match() {
    this->gicp.align(*(this->final));
    if(this->gicp.hasConverged()) {
        this->result.matrix() = gicp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}
}
}